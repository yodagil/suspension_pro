function [tr,T] = hmc_native_run(D,c)
%HMC_NATIVE_RUN Independent trim and RK4 using the optional MEX evaluator.
% [tr,T]=hmc_native_run(D,c). D has the same schema as the original importer.
% No CarSim output or measured trim state is used by this function.
% Model parameters must be configured in D/c before calling it.
if nargin<2,c=struct;end
assert(exist('hmc_mex','file')==3, ...
    'HMC:NativeMissing','Build the optional accelerator with build_hmc_mex first.');
c=defaults(c);
h=hmc_mex('new',D,c);guard=onCleanup(@()hmc_mex('delete',h)); %#ok<NASGU>
T=find_trim(h,D,c);
if (~T.root_found||~T.usable) && isfield(c,'trim_seed_y')
    c=rmfield(c,'trim_seed_y');T=find_trim(h,D,c);
end
assert(T.root_found&&T.usable,'HMC:TrimFailed', ...
    'Independent equilibrium failed, residual %.9g.',T.residual_norm);
X=T.x;
if c.corrected_initial_velocity
    % CarSim SV_VYS is a BODY-axis velocity component. The model stores
    % yaw-aligned horizontal velocities and world vertical velocity.
    phi=X(4);theta=X(5);
    body_y=sin(phi)*sin(theta)*X(33)+cos(phi)*X(1)+ ...
        sin(phi)*cos(theta)*X(10);
    change=c.vc0_mps-body_y;
    X(33)=X(33)+sin(theta)*sin(phi)*change;
    X(1)=X(1)+cos(phi)*change;
    X(10)=X(10)+cos(theta)*sin(phi)*change;
else
    X(1)=c.vc0_mps;
end
X(2)=c.r0_radps;
if strcmp(c.tire_restart,'zero'),X(17:24)=0;end
tr=hmc_mex('trajectory',h,X,T.x,c.dt,c.duration);
end

function c=defaults(c)
if nargin<1||isempty(c),c=struct;end
s=struct('speed_kmh',90,'mu',.6,'steer_deg',2.5,'wear',[0 0 0 0], ...
    'I_mA',[950 950 950 950],'spring_modes',[2 2 2 2], ...
    'trim_step_deg',.125,'trim_residual_tol',1e-6,'vc0_mps',.1, ...
    'r0_radps',.35,'dt',.001,'duration',10,'tire_restart','zero', ...
    'corrected_initial_velocity',false,'compute_trim_stability',true);
names=fieldnames(s);
for k=1:numel(names)
    if ~isfield(c,names{k}),c.(names{k})=s.(names{k});end
end
end

function T=find_trim(h,D,c)
T=struct('x',nan(34,1),'root_found',false,'usable',false, ...
    'residual_norm',Inf,'max_real_eigenvalue',NaN, ...
    'locally_stable_test',false,'reference_snapshot_used',false, ...
    'note','Own numerical equilibrium; native equation evaluation');
y=zeros(18,1);y(18)=.01;y(10:13)=.001;
angles=linspace(0,c.steer_deg,max(2,ceil(abs(c.steer_deg)/c.trim_step_deg)+1));
if isfield(c,'trim_seed_y'),y=c.trim_seed_y(:);angles=c.steer_deg;end
restore=onCleanup(@()hmc_mex('set_options',h,struct('steer_deg',c.steer_deg))); %#ok<NASGU>
for angle=angles
    hmc_mex('set_options',h,struct('steer_deg',angle));
    if angle~=0&&y(2)==0,y(2)=c.speed_kmh/3.6*deg2rad(angle)/D.vehicle.L;end
    [y,ok,res]=newton(h,y,c.trim_residual_tol);
    if ~ok,T.residual_norm=res;T.failed_angle_deg=angle;T.last_y=y;return;end
end
hmc_mex('set_options',h,struct('steer_deg',c.steer_deg));
X=hmc_mex('trimstate',h,y);[d,O]=hmc_mex('rhs',h,X);
res=max(abs(d([1:29 33:34])));
T.x=X;T.trim_unknowns=y;T.root_found=isfinite(res)&&res<c.trim_residual_tol;
T.residual_norm=res;T.vy_mps=X(1);T.r_radps=X(2);T.vx_kmh=X(33)*3.6;
T.roll_deg=rad2deg(X(4));T.pitch_deg=rad2deg(X(5));
T.rollrate_degps=rad2deg(X(11));T.pitchrate_degps=rad2deg(X(12));
T.usable=T.root_found&&all(O.Fz_raw>0)&&all(isfinite(X))&& ...
    abs(X(4))<deg2rad(40)&&X(33)>0;
if T.root_found&&c.compute_trim_stability
    ids=[1:28 33:34];steps=1e-6*max(1,abs(X(ids)));
    xx=repmat(X,1,2*numel(ids));
    for j=1:numel(ids)
        xx(ids(j),j)=xx(ids(j),j)+steps(j);
        xx(ids(j),j+numel(ids))=xx(ids(j),j+numel(ids))-steps(j);
    end
    dd=hmc_mex('rhs',h,xx);
    J=bsxfun(@rdivide,dd(ids,1:numel(ids))-dd(ids,numel(ids)+1:end),2*steps');
    T.max_real_eigenvalue=max(real(eig(J)));
    T.locally_stable_test=T.max_real_eigenvalue<1e-5;
end
end

function [y,ok,res]=newton(h,y,tol)
sc=[1;.1;.01;.03;.03;.01*ones(4,1);.01*ones(4,1);.03*ones(4,1);1];
rs=[2;1;2*ones(7,1);10*ones(4,1);100*ones(4,1);2];
z=y./sc;ok=false;res=Inf;
for iteration=1:55
    f=equations(h,z.*sc)./rs;res=max(abs(f.*rs));
    if all(isfinite(f))&&res<tol,ok=true;y=z.*sc;return;end
    if any(~isfinite(f)),break;end
    steps=1e-5*max(1,abs(z));zz=repmat(z,1,36);
    for j=1:18,zz(j,j)=zz(j,j)+steps(j);zz(j,j+18)=zz(j,j+18)-steps(j);end
    ff=equations(h,bsxfun(@times,zz,sc));
    J=bsxfun(@rdivide,bsxfun(@rdivide,ff(:,1:18)-ff(:,19:36),2*steps'),rs);
    if rcond(J)<1e-14,dz=-pinv(J)*f;else,dz=-J\f;end
    if any(~isfinite(dz)),break;end
    merit=norm(f);accepted=false;
    for a=2.^-(0:14)
        trial=z+a*dz;
        physical=abs(trial(4)*sc(4))<.6&&abs(trial(5)*sc(5))<.4&& ...
            all(abs(trial(10:13).*sc(10:13))<.5)&& ...
            all(abs(trial(14:17).*sc(14:17))<1.2);
        if ~physical,continue;end
        ft=equations(h,trial.*sc)./rs;
        if all(isfinite(ft))&&norm(ft)<merit,z=trial;accepted=true;break;end
    end
    if ~accepted,break;end
end
y=z.*sc;
end

function f=equations(h,y)
X=hmc_mex('trimstate',h,y);d=hmc_mex('rhs',h,X);
f=d([1 2 10:16 21:24 25:28 33],:);
end
