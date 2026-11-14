function outputFile=run_carsim_fullcar_region(key)
%RUN_CARSIM_FULLCAR_REGION Reproduce the original 34-state observed regions.
% Prepare contexts with prepare_three_contexts and prebuild the region MEX.
% Keys: '0', '1p5', '2p5'. All paths resolve from this repository checkout.
% The external static H is a classification reference, never the vehicle RHS.
codeDir=fileparts(mfilename('fullpath'));
stableRoot=fileparts(codeDir);root=fileparts(stableRoot);
base=fullfile(stableRoot,'comparison');
addpath(root);addpath(codeDir);
if isstring(key)&&isscalar(key),key=char(key);end
assert(ischar(key)&&any(strcmp(key,{'0','1p5','2p5'})), ...
    'HMC:RegionKey','Expected key 0, 1p5 or 2p5.');
contextFile=fullfile(base,'context',['context_' key '.mat']);
assert(isfile(contextFile),'HMC:RegionContext', ...
    'Context missing. Run prepare_three_contexts first: %s',contextFile);
S=load(contextFile,'Context');C=S.Context;
binary=which('hmc_region_mex');
expectedBinary=fullfile(codeDir,['hmc_region_mex.' mexext]);
assert(exist('hmc_region_mex','file')==3&&strcmp(binary,expectedBinary), ...
    'HMC:RegionBinary','Build the native module in this code folder first.');
% Checkpoints bind to the actual binary of this platform/build, not a
% machine-specific precompiled binary from another checkout.
C.native_binary_sha256=stable_region_sha256(binary);
outputDir=fullfile(base,'carsim_fullcar');
if ~isfolder(outputDir),mkdir(outputDir);end
if exist('OCTAVE_VERSION','builtin')~=0
    executionRuntime='GNU Octave';executionVersion=OCTAVE_VERSION;
else
    executionRuntime='MATLAB';executionVersion=version;
end
model=hmc_region_mex('new',C.Dused,C.c);
cleanup=onCleanup(@()hmc_region_mex('delete',model)); %#ok<NASGU>
cfg=C.cfg;
Htrim=reference_handling(C.T.x(1:2)',cfg.reference);
cfg.trim_inside_reference=logical(Htrim.inside);
[V,R]=ndgrid(C.grid.vc(:),C.grid.r(:));
points=[V(:),R(:)];ids=find(C.reference_mask);
assert(numel(ids)>0 && isequal(size(C.reference_mask),size(V)));
fprintf('START carsim_fullcar target=%g: reference points=%d, own trim in H=%d\n', ...
    C.command_steer_deg,numel(ids),cfg.trim_inside_reference);
started=tic;
raw=run_points(points(ids,:),model,C,cfg,base,key,'vertices');
raw.initial_points=points(ids,:);
raw.initial_grid_linear_index_1based=ids;
nh=numel(cfg.horizons);final=false(numel(V),nh);guarded=final;
final(ids,:)=raw.final==1;guarded(ids,:)=raw.guarded_final==1;
M=struct('reference',logical(C.reference_mask), ...
    'final',reshape(final,[size(V),nh]), ...
    'guarded_final',reshape(guarded,[size(V),nh]));
fm=M.final(:,:,end);
cm=fm(1:end-1,1:end-1)&fm(2:end,1:end-1)& ...
    fm(1:end-1,2:end)&fm(2:end,2:end);
candidate=find(cm);[iv,ir]=ind2sub(size(cm),candidate);
vg=C.grid.vc(:);rg=C.grid.r(:);
cp=[(vg(iv)+vg(iv+1))/2,(rg(ir)+rg(ir+1))/2];
if isempty(cp)
    centerRaw=struct();centerPass=false(0,1);
else
    centerRaw=run_points(cp,model,C,cfg,base,key,'centers');
    centerPass=centerRaw.final(:,end)==1;
    cm(candidate(~centerPass))=false;
end
[dv,dr]=ndgrid(diff(vg),diff(rg));areas=dv.*dr;
region=struct('cell_mask',cm,'cell_area',sum(areas(cm)), ...
    'candidate_cells',numel(candidate),'accepted_cells',nnz(cm), ...
    'center_points',cp,'center_pass',centerPass,'center_raw',centerRaw, ...
    'certified_continuous',false, ...
    'definition','All four vertices and the actual center pass the 10 s CarSim observed-output finite-time policy; exact cell union, no hull or shifting');
handling=struct('mask',logical(C.reference_mask),'definition',raw.reference_definition, ...
    'source','Uploaded CarSim common reference, reconstructed at its recorded actual steering angle');
caseResult=struct('wear',[25 25 0 0],'grid',C.grid, ...
    'handling',handling,'masks',M,'region',region,'raw',raw,'trim',C.T, ...
    'trim_in_reference',cfg.trim_inside_reference,'model_configuration',C.c, ...
    'classification_configuration',cfg,'calibration',C.calibration, ...
    'model_label','carsim_fullcar','source_result_job_id', ...
    210+70+21+round(C.command_steer_deg/.5)+1);
meta=struct('model','carsim_fullcar','runtime',executionRuntime, ...
    'runtime_version',executionVersion,'speed_kmh',70,'mu',.6, ...
    'wear',[25 25 0 0],'steer_target_deg',C.command_steer_deg, ...
    'model_actual_front_steer_deg',C.actual_front_steer_deg, ...
    'reference_actual_front_steer_deg',C.carsim_metadata.steer_actual_deg, ...
    'horizons_s',cfg.horizons,'integration_dt_s',cfg.dt, ...
    'observation_dt_s',cfg.observe_dt,'terminal_window_s',cfg.terminal_window, ...
    'definition','CarSim-compatible seven-observable finite-time return and H preservation; AY/yaw policy only in guarded_final', ...
    'physics','Original result/34-state HMC with speed PI and sampled 1 ms / delayed 2 ms CDC', ...
    'initialization','Original HMC lift: trim hidden states, corrected body-y velocity request, zero tire restart', ...
    'grid_coordinates','Nominal requested initial lateral-velocity/yaw-rate grid; original CarSim achieved IC frame is not recorded', ...
    'recoverable_diagnostics','Pruned trajectories are unknown for auxiliary recovery, never reported as proven unrecoverable', ...
    'certified_invariant',false,'wall_seconds',toc(started), ...
    'fresh_vertex_trajectories',numel(ids),'fresh_center_trajectories',numel(candidate), ...
    'native_binary_sha256',C.native_binary_sha256);
RESEARCH2D=struct('meta',meta,'CASE',{{caseResult}},'cfg',cfg, ...
    'original_result_signature',C.original_run_signature);
outputFile=fullfile(outputDir,['carsim_fullcar_region_' key '_core.mat']);
temporary=[outputFile '.tmp'];save(temporary,'RESEARCH2D','-v7');
[ok,message]=movefile(temporary,outputFile,'f');assert(ok,message);
summary=struct('model','carsim_fullcar','steer_target_deg',C.command_steer_deg, ...
    'reference_nodes',numel(ids),'final_nodes',sum(final,1), ...
    'accepted_cells',nnz(cm),'area',region.cell_area, ...
    'candidate_centers',numel(candidate),'elapsed_seconds',meta.wall_seconds, ...
    'trim_in_reference',cfg.trim_inside_reference,'result_file',outputFile);
write_json(fullfile(outputDir,['summary_' key '.json']),summary);
fprintf('FINISHED %s: nodes=%d/%d cells=%d area=%.10g in %.3fs\n', ...
    key,nnz(fm),numel(ids),nnz(cm),region.cell_area,meta.wall_seconds);
end

function raw=run_points(points,model,C,cfg,base,key,phase)
chunk=64;n=size(points,1);parts=cell(ceil(n/chunk),1);
checkpointDir=fullfile(base,'checkpoints',key,phase);
if ~isfolder(checkpointDir),mkdir(checkpointDir);end
for first=1:chunk:n
    last=min(n,first+chunk-1);index=floor((first-1)/chunk)+1;
    checkpoint=fullfile(checkpointDir,sprintf('batch_%04d.mat',index));
    X=HMC_FullCar_Revised('lift',points(first:last,:),C.Q,C.c,C.T);
    started=tic;
    if isfile(checkpoint)
        old=load(checkpoint,'Out','InitialStates','Config','NativeBinarySHA');
        assert(isequaln(old.InitialStates,X)&&isequaln(old.Config,cfg), ...
            'Checkpoint input mismatch; use a new computation directory.');
        assert(strcmp(old.NativeBinarySHA,C.native_binary_sha256),'Checkpoint engine mismatch.');
        Out=old.Out;
    else
        Out=hmc_region_mex('region_rollout',model,X,C.T.x,cfg);
        assert(all(isfinite(Out.final(:)))&&all(ismember(Out.final(:),[0 1])));
        assert(all(cellfun(@isempty,Out.error_message)),'Native region exception');
        InitialStates=X;Config=cfg;NativeBinarySHA=C.native_binary_sha256; %#ok<NASGU>
        tmp=[checkpoint '.tmp'];save(tmp,'Out','InitialStates','Config','NativeBinarySHA','-v7');
        [ok,message]=movefile(tmp,checkpoint,'f');assert(ok,message);
    end
    parts{index}=Out;
    progress=struct('key',key,'phase',phase,'completed',last,'total',n, ...
        'last_batch_seconds',toc(started),'model','carsim_fullcar');
    write_json(fullfile(base,['progress_' key '.json']),progress);
    fprintf('%s %s %d/%d (%.3fs)\n',key,phase,last,n,progress.last_batch_seconds);
end
raw=parts{1};
static={'final_state','trace','trace_columns','error_message','schema','dt_s', ...
    'observe_dt_s','horizons_s','terminal_tolerance','terminal_target', ...
    'window_start_inclusive','main_requires_ay','reference_definition'};
metrics=setdiff(fieldnames(raw),static);
for i=2:numel(parts)
    p=parts{i};
    for j=1:numel(metrics)
        name=metrics{j};raw.(name)=[raw.(name);p.(name)];
    end
    raw.final_state=[raw.final_state,p.final_state];
    raw.error_message=[raw.error_message;p.error_message];
    assert(isempty(p.trace),'Production computation should not save full traces.');
end
assert(size(raw.final,1)==n&&size(raw.final_state,2)==n);
end

function write_json(path,value)
temp=[path '.tmp'];fid=fopen(temp,'w');assert(fid>=0);
fprintf(fid,'%s\n',jsonencode(value));fclose(fid);
[ok,message]=movefile(temp,path,'f');assert(ok,message);
end
