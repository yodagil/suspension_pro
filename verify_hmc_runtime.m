function report = verify_hmc_runtime(userOptions)
%VERIFY_HMC_RUNTIME Check pure-MATLAB and native equation equivalence.
%   report = verify_hmc_runtime;
%   report = verify_hmc_runtime(struct('require_mex',true));
%
% Runs baseline and revised configurations at 50/.5/0, 70/.6/2.5, and
% 90/.6/2.5. Each variant uses its configured independent physical inputs
% and, when enabled, hmc_apply_calibration. No CarSim trajectory is read.
%
% Checks all 34 state derivatives, vectorized trim-state/RHS evaluation,
% initial-coordinate conversion, held damper-force evaluation, and a 0.05 s
% RK4 trajectory. The six requested output channels are checked separately.
% Revised checks include MF6.2 trail, CC_FX sign, and effective Euler-rate
% damping through the selected model/calibration profile.
%
% The tolerance is 1e-7 for max(abs(A-B)./(1+max(abs(A),abs(B)))).
% A missing MEX reports SKIPPED, never PASS. Run build_hmc_mex first.
% A discrepancy asserts with its variant, condition and failed check.
% This checks implementation equivalence; it is not CarSim validation.

if nargin<1||isempty(userOptions),userOptions=struct;end
assert(isstruct(userOptions)&&isscalar(userOptions), ...
    'HMC:VerifyOptions','Options must be a scalar structure.');
requireMex=false;
if isfield(userOptions,'require_mex')
    requireMex=logical(userOptions.require_mex);
    assert(isscalar(requireMex),'require_mex must be scalar.');
    userOptions=rmfield(userOptions,'require_mex');
end
report=struct('schema','HMC_RUNTIME_EQUIVALENCE_V1', ...
    'status','not_run','executed',false,'passed',false,'skipped',false, ...
    'scaled_tolerance',1e-7,'duration_s',.05,'case_count',0, ...
    'case_results',[],'reason','', ...
    'scope','MATLAB/native implementation equivalence; no CarSim validation');
if exist('hmc_mex','file')~=3
    report.status='skipped';report.skipped=true;
    report.reason=['Native MEX is unavailable. Run build_hmc_mex and ', ...
        'then verify_hmc_runtime again; equivalence has not been verified.'];
    fprintf('SKIPPED: %s\n',report.reason);
    if requireMex,error('HMC:VerificationSkipped','%s',report.reason);end
    return;
end

userOptions.make_plot=false;userOptions.save_results=false;
userOptions.duration=.05;userOptions.dt=.001;
userOptions.progress_stride=0;userOptions.compute_trim_stability=false;
base=HMC_FullCar_Revised('options',userOptions);
S=load(base.data_file,'D');
assert(isfield(S,'D'),'HMC:VerifyInput','Data MAT must contain D.');
conditions=[50 .5 0;70 .6 2.5;90 .6 2.5];
variants={'baseline','revised'};
results=cell(6,1);job=0;
for iv=1:numel(variants)
    for ic=1:size(conditions,1)
        job=job+1;
        supplied=userOptions;supplied.variant=variants{iv};
        supplied.speed_kmh=conditions(ic,1);supplied.mu=conditions(ic,2);
        supplied.steer_deg=conditions(ic,3);
        c=HMC_FullCar_Revised('options',supplied);
        [Dused,profile]=hmc_apply_calibration(S.D,c);
        Q=HMC_FullCar_Revised('configuration',c,c.wear,Dused);
        fprintf('Verifying %s: %g km/h, mu=%g, steer=%g deg...\n', ...
            c.variant,c.speed_kmh,c.mu,c.steer_deg);
        results{job}=verify_case(Dused,Q,c,profile,report.scaled_tolerance);
    end
end
report.case_results=vertcat(results{:});
report.case_count=numel(results);report.executed=true;
report.passed=true;report.status='passed';
fprintf('PASS: %d baseline/revised cases; all checked differences <= %.1e scaled.\n', ...
    report.case_count,report.scaled_tolerance);
fprintf('This result verifies the two implementations, not new CarSim conditions.\n');
if nargout==0,disp(struct2table(report.case_results));end
end

function out=verify_case(D,Q,c,profile,tol)
label=sprintf('%s %g km/h',c.variant,c.speed_kmh);
% Obtain equilibrium only from the independent model. No reference states.
[nativeTrace,T]=hmc_native_run(D,c);
h=hmc_mex('new',D,c);
cleanup=onCleanup(@()hmc_mex('delete',h)); %#ok<NASGU>

% The MATLAB trim-state mapper is exercised on several complete unknown
% vectors, including nonzero lateral/longitudinal tire deformation states.
y=T.trim_unknowns(:);
yscale=[.02;.01;.0001;.0005;.0005; ...
    .00005*ones(4,1);.0001*ones(4,1);.0005*ones(4,1);.001];
Y=[y,bsxfun(@plus,y,bsxfun(@times,yscale,sin((1:18)'*(1:4))))];
Xm=HMC_FullCar_Revised('trimstate',Y,Q);
Xn=hmc_mex('trimstate',h,Y);
trimError=check_equal(Xm,Xn,tol,[label ' vectorized trimstate']);

X0=HMC_FullCar_Revised('lift',[c.vc0_mps c.r0_radps],Q,c,T);
icError=check_equal(X0,nativeTrace.state(:,1),tol,[label ' initial coordinates']);

% Deterministic, small full-state perturbations exercise every state without
% relying only on zero entries of the equilibrium or restart state.
xscale=[.03;.02;.0002;.001;.001;.0001*ones(4,1); ...
    .01;.03;.02;.01*ones(4,1);.001*ones(4,1);.001*ones(4,1); ...
    .01*ones(4,1);.1;.02;.1;.1;.1;.001];
perturbed=bsxfun(@plus,T.x,bsxfun(@times,xscale,sin((1:34)'*(1:4))));
sample=[2 6 11 26 size(nativeTrace.state,2)];
X=[T.x,X0,nativeTrace.state(:,sample),perturbed];
dm=HMC_FullCar_Revised('rhs',X,Q);
dn=hmc_mex('rhs',h,X);
rhsBatchError=check_equal(dm,dn,tol,[label ' batched RHS']);
rhsSingleError=0;forceError=0;heldError=0;
forceFields={'ay','ax','throttle','Fz_raw','Fz','Fx','Fy','Mz','My', ...
    'delta','gamma','jounce','spring_force','damper_force','jacking_force'};
for k=1:size(X,2)
    [d1,O1]=HMC_FullCar_Revised('rhs',X(:,k),Q);
    [d2,O2]=hmc_mex('rhs',h,X(:,k));
    rhsSingleError=max(rhsSingleError, ...
        check_equal(d1,d2,tol,[label ' scalar RHS']));
    check_equal(d1,dm(:,k),tol,[label ' MATLAB scalar/batch consistency']);
    for j=1:numel(forceFields)
        key=forceFields{j};
        forceError=max(forceError, ...
            check_equal(O1.(key),O2.(key),tol,[label ' diagnostic ' key]));
    end
    % An imposed held command tests the delayed-force RHS path directly.
    held=O2.damper_force+[11;-7;5;-3];
    dh1=HMC_FullCar_Revised('rhs',X(:,k),Q,held);
    dh2=hmc_mex('rhs',h,X(:,k),held);
    heldError=max(heldError,check_equal(dh1,dh2,tol,[label ' held damper RHS']));
end

matlabTrace=HMC_FullCar_Revised('trajectory',X0,Q,c,T);
nativeDirect=hmc_mex('trajectory',h,X0,T.x,c.dt,c.duration);
timeError=check_equal(matlabTrace.time_s,nativeDirect.time_s,tol,[label ' sample times']);
stateError=check_equal(matlabTrace.state,nativeDirect.state,tol,[label ' RK4 states']);
check_equal(nativeTrace.state,nativeDirect.state,tol,[label ' native wrapper trajectory']);
for key={'ay_mps2','Fz_raw_N','spring_force_N','damper_force_N'}
    field=key{1};
    check_equal(matlabTrace.(field),nativeDirect.(field),tol,[label ' trajectory ' field]);
end

% Explicit six-signal mapping: vy, body p, body r, vx, body q, roll angle.
ids=[1 11 2 33 12 4];scale=[1;180/pi;1;1;180/pi;180/pi];
ym=bsxfun(@times,matlabTrace.state(ids,:),scale);
yn=bsxfun(@times,nativeDirect.state(ids,:),scale);
sixError=check_equal(ym,yn,tol,[label ' six requested signals']);
out=struct('Variant',c.variant,'Speed_kmh',c.speed_kmh, ...
    'Mu',c.mu,'Steer_deg',c.steer_deg,'Passed',true, ...
    'TrimResidual',T.residual_norm,'TrimStateScaledError',trimError, ...
    'InitialStateScaledError',icError,'RHSBatchScaledError',rhsBatchError, ...
    'RHSScalarScaledError',rhsSingleError,'ForceScaledError',forceError, ...
    'HeldDamperScaledError',heldError,'SampleTimeScaledError',timeError, ...
    'TrajectoryStateScaledError',stateError,'SixSignalScaledError',sixError, ...
    'CorrectedCCFxSign',c.corrected_cc_fx_sign, ...
    'MF62TrailCorrection',c.mf62_zero_camber_trail, ...
    'ExtraRollDamping_Nms',Q.extra_roll_damping_Nms, ...
    'ExtraPitchDamping_Nms',Q.extra_pitch_damping_Nms);
if isfield(profile,'enabled'),out.CalibrationEnabled=logical(profile.enabled);
else,out.CalibrationEnabled=logical(c.calibration_enabled);end
end

function err=check_equal(A,B,tol,label)
assert(isequal(size(A),size(B)),'HMC:RuntimeShape', ...
    '%s: sizes differ (%s versus %s).',label,mat2str(size(A)),mat2str(size(B)));
assert(all(isfinite(A(:)))&&all(isfinite(B(:))),'HMC:RuntimeNonfinite', ...
    '%s contains nonfinite values.',label);
relative=abs(A-B)./(1+max(abs(A),abs(B)));
err=max(relative(:));if isempty(err),err=0;end
assert(err<=tol,'HMC:RuntimeMismatch', ...
    '%s failed: scaled difference %.12g exceeds %.12g.',label,err,tol);
end
