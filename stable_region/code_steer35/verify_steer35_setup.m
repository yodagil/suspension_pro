function Report=verify_steer35_setup(ids)
% Verify appended inputs and frozen engines without computing region grids.
if nargin<1,ids=[1051 1090 1200];end
code=fileparts(mfilename('fullpath'));stable=fileparts(code);root=fileparts(stable);
addpath(root);addpath(code);
assert(strcmp(OCTAVE_VERSION,'10.3.0'));
assert(strcmp(stable_region_sha256(which('hmc_mex')), ...
    'da8175679e57d47086f23a645806272eb613c9cc3695f8684572deea1b99a955'));
assert(strcmp(stable_region_sha256(which('hmc_region_mex')), ...
    '0218babeb33e18b403cbf2113be53d328bc0da58546bea82304b1696bc617194'));
for id=ids(:)'
    s=load(fullfile(stable,'carsim_fullcar_steer35_150','context',sprintf('case_%04d.mat',id)),'Context');C=s.Context;
    assert(C.job_id==id&&C.command_steer_deg==3.5&&C.c.steer_deg==3.5);
    assert(~C.source_result_available&&isempty(C.source_result_file));
    assert(C.T.root_found&&C.T.usable&&all(isfinite(C.T.x)));
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
end
Report=struct('status','pass','checked_ids',ids,'command_steer_deg',3.5, ...
    'bitwise_rhs_and_observations_match',true,'reference_nodes_per_case',7381, ...
    'full_region_rollouts',0,'original_native_and_region_binary_bytes_preserved',true);
disp(Report);
end
