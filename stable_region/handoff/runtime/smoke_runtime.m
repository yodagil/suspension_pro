% No full region trajectories: validate the packaged model/context and MEX.
scriptRoot=fileparts(mfilename('fullpath'));
root=getenv('HMC_PROJECT_ROOT');if isempty(root),root=scriptRoot;end
while ~isfile(fullfile(root,'HMC_FullCar_Revised.m'))
    parent=fileparts(root);assert(~strcmp(parent,root),'Set HMC_PROJECT_ROOT.');root=parent;
end
addpath(root);addpath(fullfile(root,'stable_region','code'));
context=fullfile(root,'stable_region','carsim_fullcar_1050','context','case_0526.mat');
saved=load(context,'Context');C=saved.Context;
assert(C.job_id==526&&C.T.root_found&&C.T.usable&&strcmp(OCTAVE_VERSION,'10.3.0'));
assert(strcmp(which('hmc_mex'),fullfile(root,'hmc_mex.mex')));
assert(strcmp(which('hmc_region_mex'),fullfile(root,'stable_region','code','hmc_region_mex.mex')));
assert(strcmp(stable_region_sha256(which('hmc_region_mex')), ...
 '0218babeb33e18b403cbf2113be53d328bc0da58546bea82304b1696bc617194'));
h1=hmc_mex('new',C.Dused,C.c);h2=hmc_region_mex('new',C.Dused,C.c);
[d1,o1]=hmc_mex('rhs',h1,C.T.x);[d2,o2]=hmc_region_mex('rhs',h2,C.T.x);
assert(isequaln(d1,d2)&&isequaln(o1,o2)&&all(isfinite(d1)));
X=HMC_FullCar_Revised('lift',[0 0;.1 .35],C.Q,C.c,C.T);
assert(isequal(size(X),[34 2])&&all(isfinite(X(:)))&&all(all(X(17:24,:)==0)));
[V,R]=ndgrid(C.grid.vc,C.grid.r);H=reference_handling([V(:),R(:)],C.cfg.reference);
assert(isequal(reshape(logical(H.inside),size(V)),logical(C.reference_mask)));
Htrim=reference_handling(C.T.x(1:2)',C.cfg.reference);
assert(logical(Htrim.inside)==logical(C.cfg.trim_inside_reference));
hmc_mex('delete',h1);hmc_region_mex('delete',h2);
fprintf('PASS: context526, exact frozen MEX, bitwise RHS/observations, two lifted points, all7381 H nodes; full rollouts=0.\n');
