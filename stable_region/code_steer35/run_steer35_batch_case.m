function outputFile=run_steer35_batch_case(jobID)
%RUN_REGION_BATCH_CASE Exact finite-time region for one appended steering3.5deg condition.
% Same frozen34-state physical kernel and seven-observable policy as the
% selected comparison. All batch cases use their own actual trim steer in H.
codeDir=fileparts(mfilename('fullpath'));
stableRoot=fileparts(codeDir);root=fileparts(stableRoot);
base=fullfile(stableRoot,'carsim_fullcar_steer35_150');
addpath(root);addpath(codeDir);
assert(isnumeric(jobID)&&isscalar(jobID)&&isfinite(jobID)&&jobID==fix(jobID)&& ...
    jobID>=1051&&jobID<=1200,'HMC:BatchID','Expected an integer appended job ID1051..1200.');
key=sprintf('case_%04d',jobID);
contextFile=fullfile(base,'context',[key '.mat']);
assert(isfile(contextFile),'HMC:BatchContext','Run prepare_steer35_batch first.');
saved=load(contextFile,'Context');C=saved.Context;
assert(C.job_id==jobID&&strcmp(C.status,'prepared'));
assert_original_sources(root,C.original_run_signature);
C.context_file_sha256=stable_region_sha256(contextFile);
binary=which('hmc_region_mex');
assert(exist('hmc_region_mex','file')==3&& ...
    strcmp(binary,fullfile(codeDir,['hmc_region_mex.' mexext])), ...
    'HMC:BatchBinary','Build the native region module in the code folder.');
C.native_binary_sha256=stable_region_sha256(binary);
binding=jsondecode(fileread(fullfile(stableRoot,'comparison','validation','binary_binding.json')));
assert(strcmp(stable_region_sha256(fullfile(codeDir,'hmc_region_mex.cpp')), ...
    binding.cpp_sha256),'HMC:BatchKernel','Region observer source differs from the verified engine.');
cfg=C.cfg;
Htrim=reference_handling(C.T.x(1:2)',cfg.reference);
assert(logical(Htrim.inside)==logical(cfg.trim_inside_reference));
assert(C.main_region_known_empty==~logical(cfg.trim_inside_reference));
[V,R]=ndgrid(C.grid.vc(:),C.grid.r(:));points=[V(:),R(:)];
ids=find(C.reference_mask);nh=numel(cfg.horizons);
assert(isequal(size(C.reference_mask),[61 121])&&nh==2&&isequal(cfg.horizons(:)',[5 10]));
fprintf('START globalID%d/1200 wear%d %d/30: H=%d trim_in_H=%d\n', ...
    jobID,C.wear_case_id,C.condition_id,numel(ids),cfg.trim_inside_reference);
started=tic;actualVertices=0;actualCenters=0;
if C.main_region_known_empty||isempty(ids)
    raw=uncomputed_main_false(numel(ids),nh);
    completionKind='trim_outside_reference';
    if isempty(ids),completionKind='empty_reference_grid';end
else
    model=hmc_region_mex('new',C.Dused,C.c);
    cleanup=onCleanup(@()hmc_region_mex('delete',model)); %#ok<NASGU>
    raw=run_points(points(ids,:),model,C,cfg,base,key,'vertices');
    actualVertices=numel(ids);completionKind='integrated';
end
raw.initial_points=points(ids,:);raw.initial_grid_linear_index_1based=ids;
final=false(numel(V),nh);guarded=final;
final(ids,:)=raw.final==1;guarded(ids,:)=raw.guarded_final==1;
M=struct('reference',logical(C.reference_mask), ...
    'final',reshape(final,[size(V),nh]),'guarded_final',reshape(guarded,[size(V),nh]));
fm=M.final(:,:,end);
cm=fm(1:end-1,1:end-1)&fm(2:end,1:end-1)&fm(1:end-1,2:end)&fm(2:end,2:end);
candidate=find(cm);[iv,ir]=ind2sub(size(cm),candidate);
vg=C.grid.vc(:);rg=C.grid.r(:);cp=[(vg(iv)+vg(iv+1))/2,(rg(ir)+rg(ir+1))/2];
if isempty(cp)
    centerRaw=struct();centerPass=false(0,1);
else
    centerRaw=run_points(cp,model,C,cfg,base,key,'centers');
    actualCenters=size(cp,1);centerPass=centerRaw.final(:,end)==1;
    cm(candidate(~centerPass))=false;
end
[dv,dr]=ndgrid(diff(vg),diff(rg));areas=dv.*dr;
referenceTruncated=touches_boundary(C.reference_mask);
stableTruncated=touches_boundary(cm);
region=struct('cell_mask',cm,'cell_area',sum(areas(cm)), ...
    'candidate_cells',numel(candidate),'accepted_cells',nnz(cm), ...
    'center_points',cp,'center_pass',centerPass,'center_raw',centerRaw, ...
    'certified_continuous',false,'grid_truncated',stableTruncated, ...
    'definition','Exact union of cells with four accepted vertices and an independently integrated accepted center; finite10s policy');
handling=struct('mask',logical(C.reference_mask), ...
    'source','Uniform external static MF H at own model mean actual front trim steering', ...
    'reference_mode',C.reference_mode,'grid_truncated',referenceTruncated);
caseResult=struct('job_id',jobID,'condition_id',C.condition_id, ...
    'wear_case_id',C.wear_case_id,'wear',C.wear,'grid',C.grid, ...
    'handling',handling,'masks',M,'region',region,'raw',raw,'trim',C.T, ...
    'trim_in_reference',cfg.trim_inside_reference,'model_configuration',C.c, ...
    'classification_configuration',cfg,'calibration',C.calibration, ...
    'model_label','carsim_fullcar','source_result_job_id',[], ...
    'source_result_file',C.source_result_file,'source_result_available',false);
if exist('OCTAVE_VERSION','builtin')~=0
    executionRuntime='GNU Octave';executionVersion=OCTAVE_VERSION;
else
    executionRuntime='MATLAB';executionVersion=version;
end
meta=struct('model','carsim_fullcar','runtime',executionRuntime, ...
    'runtime_version',executionVersion,'job_id',jobID,'condition_id',C.condition_id, ...
    'wear_case_id',C.wear_case_id,'speed_kmh',C.speed_kmh,'mu',C.mu, ...
    'wear',C.wear,'steer_target_deg',C.command_steer_deg, ...
    'model_actual_front_steer_deg',C.actual_front_steer_deg, ...
    'reference_mode',C.reference_mode,'reference_actual_front_steer_deg',C.actual_front_steer_deg, ...
    'horizons_s',cfg.horizons,'integration_dt_s',cfg.dt, ...
    'observation_dt_s',cfg.observe_dt,'terminal_window_s',cfg.terminal_window, ...
    'completion_kind',completionKind,'status','complete', ...
    'definition','Seven observed finite-time return channels, physical-output and whole-path H preservation; optional AY/yaw guards separate', ...
    'physics','Original result/34-state HMC with speed PI, sampled1ms/delayed2ms CDC', ...
    'initialization','Original lift with own hidden trim states, corrected body-y velocity, zero tire restart', ...
    'grid_coordinates','Nominal requested initial lateral-velocity/yaw-rate command grid', ...
    'outside_grid','Uncomputed; a truncated grid does not classify its exterior', ...
    'outside_reference','Not rolled out; excluded from the main domain, not demonstrated dynamically unstable', ...
    'recoverable_diagnostics','Pruned or skipped trajectory auxiliary diagnostics are unknown/NaN', ...
    'certified_invariant',false,'wall_seconds',toc(started), ...
    'fresh_vertex_trajectories',actualVertices,'fresh_center_trajectories',actualCenters, ...
    'reference_grid_truncated',referenceTruncated,'stable_grid_truncated',stableTruncated, ...
    'batch_signature',C.batch_signature,'context_sha256',C.context_file_sha256, ...
    'native_binary_sha256',C.native_binary_sha256,'kernel_source_sha256',binding.cpp_sha256);
RESEARCH2D=struct('meta',meta,'CASE',{{caseResult}},'cfg',cfg, ...
    'original_result_signature',C.original_run_signature);
outputDir=fullfile(base,'regions');if ~isfolder(outputDir),mkdir(outputDir);end
outputFile=fullfile(outputDir,[key '.mat']);temporary=[outputFile '.tmp'];
save(temporary,'RESEARCH2D','-v7');
[ok,message]=movefile(temporary,outputFile,'f');assert(ok,message);
fprintf('FINISHED globalID%d/1200 (%d/30 wear%d): nodes=%d cells=%d area=%.10g in%.3fs [%s]\n', ...
    jobID,C.condition_id,C.wear_case_id,nnz(fm),nnz(cm),region.cell_area,meta.wall_seconds,completionKind);
end

function yes=touches_boundary(mask)
yes=any(mask(1,:))||any(mask(end,:))||any(mask(:,1))||any(mask(:,end));
end

function raw=uncomputed_main_false(n,nh)
raw=struct('final',false(n,nh),'guarded_final',false(n,nh), ...
    'recoverable',NaN(n,nh),'model_safe',NaN(n,nh),'yaw_ok',NaN(n,nh), ...
    'ay_ok',NaN(n,nh),'terminal_error',NaN(n,nh), ...
    'trajectory_evaluated',false(n,1),'full_horizon_evaluated',false(n,nh), ...
    'not_integrated_reason','Main necessary condition trim_in_H is false, or reference grid is empty; auxiliary dynamics unknown');
end

function assert_original_sources(root,S)
assert(strcmp(stable_region_sha256(fullfile(root,'HMC_FullCar_Revised.m')),S.model_sha256));
assert(strcmp(stable_region_sha256(fullfile(root,'data','hmc_data.mat')),S.data_sha256));
names={'hmc_apply_calibration.m','hmc_native_run.m','hmc_mex.cpp','hmc_native.cpp','tire_fields.inc','active_calibration_file'};
for k=1:numel(names)
    idx=find(strcmp({S.dependencies.name},names{k}));assert(isscalar(idx));
    name=names{k};if strcmp(name,'active_calibration_file'),name='calibration_active.json';end
    assert(strcmp(stable_region_sha256(fullfile(root,name)),S.dependencies(idx).sha256), ...
        'HMC:BatchSourceChanged','Original model dependency differs: %s',name);
end
end

function raw=run_points(points,model,C,cfg,base,key,phase)
chunk=128;n=size(points,1);parts=cell(ceil(n/chunk),1);
checkpointDir=fullfile(base,'checkpoints',key,phase);
if ~isfolder(checkpointDir),mkdir(checkpointDir);end
for first=1:chunk:n
    last=min(n,first+chunk-1);index=floor((first-1)/chunk)+1;
    checkpoint=fullfile(checkpointDir,sprintf('batch_%04d.mat',index));
    X=HMC_FullCar_Revised('lift',points(first:last,:),C.Q,C.c,C.T);
    started=tic;
    if isfile(checkpoint)
        old=load(checkpoint,'Out','InitialStates','Config','NativeBinarySHA','ContextSHA');
        assert(isequaln(old.InitialStates,X)&&isequaln(old.Config,cfg), ...
            'Checkpoint input mismatch; use a new computation directory.');
        assert(strcmp(old.NativeBinarySHA,C.native_binary_sha256),'Checkpoint engine mismatch.');
        assert(strcmp(old.ContextSHA,C.context_file_sha256),'Checkpoint context mismatch.');
        Out=old.Out;
    else
        Out=hmc_region_mex('region_rollout',model,X,C.T.x,cfg);
        assert(all(isfinite(Out.final(:)))&&all(ismember(Out.final(:),[0 1])));
        assert(all(cellfun(@isempty,Out.error_message)),'Native region exception');
        InitialStates=X;Config=cfg;NativeBinarySHA=C.native_binary_sha256;ContextSHA=C.context_file_sha256; %#ok<NASGU>
        tmp=[checkpoint '.tmp'];save(tmp,'Out','InitialStates','Config','NativeBinarySHA','ContextSHA','-v7');
        [ok,message]=movefile(tmp,checkpoint,'f');assert(ok,message);
    end
    parts{index}=Out;
    progress=struct('key',key,'phase',phase,'completed',last,'total',n, ...
        'last_batch_seconds',toc(started),'model','carsim_fullcar');
    write_json(fullfile(base,'active_case.json'),progress);
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
