function contextFiles=prepare_three_contexts
%PREPARE_THREE_CONTEXTS Original HMC contexts for CarSim observed-region tests.
% Reads the original1050-run options and archived CarSim core/Qref inputs.
% Only data/calibration locations are relocated to this repository checkout.
% This performs the original native trim plus its5ms preparation trajectory;
% it does not run the full region grid or alter the 34-state vehicle model.
% Build root hmc_mex and code/hmc_region_mex before the production workflow.
codeDir=fileparts(mfilename('fullpath'));
stableRoot=fileparts(codeDir);root=fileparts(stableRoot);
base=fullfile(stableRoot,'comparison');
referenceDir=fullfile(base,'reference_core');contextDir=fullfile(base,'context');
addpath(root);addpath(codeDir);
assert(exist('hmc_mex','file')==3&& ...
    strcmp(which('hmc_mex'),fullfile(root,['hmc_mex.' mexext])), ...
    'HMC:RegionNativeMissing', ...
    'Build and load the original hmc_mex from this repository root.');
runFile=fullfile(root,'result','run_configuration.mat');
assert(isfile(runFile),'HMC:RegionRunMissing','Original run configuration missing: %s',runFile);
Run=load(runFile,'Config','Signature');
assert_source_signature(root,Run.Signature);
if ~isfolder(contextDir),mkdir(contextDir);end
keys={'0','1p5','2p5'};angles=[0 1.5 2.5];contextFiles=cell(3,1);
for k=1:3
    coreFile=fullfile(referenceDir,['CarSim_region_' keys{k} '_core.mat']);
    referenceFile=fullfile(referenceDir,['Qref_' keys{k} '.mat']);
    assert(isfile(coreFile)&&isfile(referenceFile),'HMC:RegionReferenceMissing', ...
        'Archived CarSim core and Qref are required for target %s.',keys{k});
    src=load(coreFile,'RESEARCH2D');Ref=src.RESEARCH2D;
    if iscell(Ref.CASE),Cref=Ref.CASE{1};else,Cref=Ref.CASE;end
    static=load(referenceFile,'Qref');
    opts=Run.Config.options;
    opts.data_file=fullfile(root,'data','hmc_data.mat');
    opts.calibration_file=fullfile(root,'calibration_active.json');
    opts.speed_kmh=70;opts.mu=.6;opts.steer_deg=angles(k);opts.wear=[25 25 0 0];
    opts.dt=.0005;opts.duration=.005;opts.make_plot=false;opts.save_results=false;opts.progress_stride=0;
    c=HMC_FullCar_Revised('options',opts);
    data=load(c.data_file,'D');[Dused,profile]=hmc_apply_calibration(data.D,c);
    [~,T]=hmc_native_run(Dused,c);c.duration=10;
    Q=HMC_FullCar_Revised('configuration',c,c.wear,Dused);
    h=hmc_mex('new',Dused,c);nativeGuard=onCleanup(@()hmc_mex('delete',h));
    [~,O]=hmc_mex('rhs',h,T.x);clear nativeGuard;
    actual_front_steer_deg=mean(O.delta(1:2))*180/pi;
    cfg=struct('dt',.0005,'observe_dt',.001,'horizons',[5 10], ...
        'terminal_window',Ref.cfg.lastWindow_s,'policy_g',9.81,'mu',.6, ...
        'yaw_factor',Ref.cfg.yawPolicyFactor,'ay_factor',Ref.cfg.ayPolicyFactor, ...
        'LTR_limit',Ref.cfg.LTRLimit,'roll_limit_rad',Ref.cfg.rollAbsoluteLimit_deg*pi/180, ...
        'early_reject_final',true,'window_start_inclusive',true, ...
        'main_requires_ay',false,'threads',4,'trace_stride',0,'reference',static.Qref);
    tt=Ref.cfg.terminalTol;
    cfg.terminal_tolerance=[tt.vy_mps tt.r_radps tt.roll_deg tt.pitch_deg ...
        tt.rollrate_degps tt.pitchrate_degps tt.vx_kmh];
    Context=struct('key',keys{k},'command_steer_deg',angles(k), ...
        'actual_front_steer_deg',actual_front_steer_deg,'c',c,'Q',Q,'T',T,'Dused',Dused, ...
        'calibration',profile,'cfg',cfg,'grid',Cref.grid,'reference_mask',Cref.handling.mask, ...
        'carsim_trim',Cref.trim,'carsim_metadata',Ref.meta,'original_run_signature',Run.Signature);
    contextFiles{k}=fullfile(contextDir,['context_' keys{k} '.mat']);
    save(contextFiles{k},'Context','-v7');
    fprintf('Context target=%g actual_model=%g actual_CarSim=%g trim_vy=%g trim_r=%g\n', ...
        angles(k),actual_front_steer_deg,Ref.meta.steer_actual_deg,T.x(1),T.x(2));
end
end

function assert_source_signature(root,Signature)
% Keep physics, physical data and active calibration anchored to the1050 run.
assert(strcmp(stable_region_sha256(fullfile(root,'HMC_FullCar_Revised.m')), ...
    Signature.model_sha256),'HMC:RegionModelChanged','Model source differs from the1050-run signature.');
assert(strcmp(stable_region_sha256(fullfile(root,'data','hmc_data.mat')), ...
    Signature.data_sha256),'HMC:RegionDataChanged','Physical data differs from the1050-run signature.');
assert(isfield(Signature,'dependencies'),'HMC:RegionCalibrationSignature', ...
    'Original run signature lacks its calibration dependency.');
dependencies=Signature.dependencies;
sourceNames={'hmc_apply_calibration.m','hmc_native_run.m','hmc_mex.cpp', ...
    'hmc_native.cpp','tire_fields.inc'};
for k=1:numel(sourceNames)
    index=find(strcmp({dependencies.name},sourceNames{k}));
    assert(isscalar(index),'HMC:RegionSourceSignature', ...
        'Missing or duplicate original source fingerprint: %s',sourceNames{k});
    assert(strcmp(stable_region_sha256(fullfile(root,sourceNames{k})), ...
        dependencies(index).sha256),'HMC:RegionDependencyChanged', ...
        'Source differs from the1050-run signature: %s',sourceNames{k});
end
% A fresh platform-specific MEX may have a different binary SHA; its source
% inputs above remain bound to the original1050-run implementation.
index=find(strcmp({dependencies.name},'active_calibration_file'));
assert(isscalar(index),'HMC:RegionCalibrationSignature', ...
    'Expected exactly one active calibration fingerprint in the original signature.');
assert(strcmp(stable_region_sha256(fullfile(root,'calibration_active.json')), ...
    dependencies(index).sha256),'HMC:RegionCalibrationChanged', ...
    'Active calibration differs from the1050-run signature.');
end
