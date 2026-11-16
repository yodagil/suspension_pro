function Report=prepare_steer35_batch(jobIDs)
%PREPARE_STEER35_BATCH Append150 original-model contexts at command3.5deg.
% Original IDs1..1050 and their configuration/files remain unchanged.
% New global IDs1051..1200 map to extension indices1..150 in wear/speed/mu
% order. condition_id is1..30 within each extension wear, not a legacy ID.
% This prepares own equilibria and5ms probe runs, never full region grids.
codeDir=fileparts(mfilename('fullpath'));
stableRoot=fileparts(codeDir);root=fileparts(stableRoot);
base=fullfile(stableRoot,'carsim_fullcar_steer35_150');
contextDir=fullfile(base,'context');errorDir=fullfile(base,'preparation_errors');
addpath(root);addpath(codeDir);
assert(exist('hmc_mex','file')==3&& ...
    strcmp(which('hmc_mex'),fullfile(root,['hmc_mex.' mexext])), ...
    'HMC:Steer35Native','Load the unchanged original hmc_mex in the repository root.');
Run=load(fullfile(root,'result','run_configuration.mat'),'Config','Signature');
assert_original_sources(root,Run.Signature);
assert(isequal(Run.Config.wear_cases,[0 0 0 0;25 25 0 0;50 50 25 25; ...
    0 0 25 25;25 25 50 50]),'HMC:Steer35WearOrder','Original wear order changed.');
assert(isequal(Run.Config.speeds_kmh(:)',50:10:100)&& ...
    isequal(Run.Config.mus(:)',[.9 .8 .7 .6 .5])&& ...
    isequal(Run.Config.steers_deg(:)',0:.5:3), ...
    'HMC:Steer35OriginalOrder','Original1050 operating-condition order changed.');
legacyFile=fullfile(stableRoot,'carsim_fullcar_1050','batch_configuration.mat');
previous=load(legacyFile,'Batch');Legacy=previous.Batch;
assert(Legacy.total_cases==1050&&Legacy.conditions_per_wear==210&& ...
    strcmp(Legacy.reference_mode,'model_actual_front_trim'), ...
    'HMC:Steer35LegacyBatch','Expected the original1050-case batch.');
assert(isequaln(Legacy.InputSignature.original_run_signature,Run.Signature), ...
    'HMC:Steer35LegacySignature','Legacy batch and original run signatures differ.');
ExtensionConfig=Run.Config;ExtensionConfig.steers_deg=3.5;
referenceBase=fullfile(stableRoot,'comparison','reference_core','Qref_0.mat');
S=load(referenceBase,'Qref');Qbase=S.Qref;
referenceSHA=stable_region_sha256(referenceBase);
assert(strcmp(referenceSHA,Legacy.InputSignature.reference_base_sha256), ...
    'HMC:Steer35Reference','The original static reference parameters changed.');
kernelSourceSHA=stable_region_sha256(fullfile(codeDir,'hmc_region_mex.cpp'));
kernelProof=jsondecode(fileread(fullfile(stableRoot,'comparison','validation','binary_binding.json')));
assert(strcmp(kernelSourceSHA,kernelProof.cpp_sha256)&& ...
    strcmp(kernelSourceSHA,Legacy.InputSignature.kernel_source_sha256), ...
    'HMC:Steer35Kernel','The original frozen region kernel source changed.');
regionBinary=fullfile(codeDir,['hmc_region_mex.' mexext]);
assert(isfile(regionBinary)&&strcmp(stable_region_sha256(regionBinary),kernelProof.mex_sha256), ...
    'HMC:Steer35RegionBinary','Use the exact verified frozen region MEX.');
nativeSHA=stable_region_sha256(which('hmc_mex'));
nativeIndex=find(strcmp({Run.Signature.dependencies.name},'available_hmc_mex_binary'));
assert(isscalar(nativeIndex)&&strcmp(nativeSHA,Run.Signature.dependencies(nativeIndex).sha256), ...
    'HMC:Steer35NativeBinary','Use the exact original native MEX.');
referenceHelper=fullfile(codeDir,'reference_handling.m');
helperSHA=stable_region_sha256(referenceHelper);
assert(strcmp(helperSHA,stable_region_sha256(fullfile(stableRoot,'code','reference_handling.m'))), ...
    'HMC:Steer35ReferenceHelper','The external H evaluator must remain unchanged.');
grid=struct('vc',(-30:30)/10,'r',(-60:60)/40);
policy=struct('dt',.0005,'observe_dt',.001,'horizons',[5 10], ...
    'terminal_window',1,'policy_g',9.81,'yaw_factor',.85,'ay_factor',1, ...
    'LTR_limit',1,'roll_limit_rad',40*pi/180,'early_reject_final',true, ...
    'window_start_inclusive',true,'main_requires_ay',false,'threads',4, ...
    'trace_stride',0,'terminal_tolerance',[.1 .005 .2 .2 .5 .5 .5]);
assert(isequaln(grid,Legacy.grid)&&isequaln(policy,Legacy.policy), ...
    'HMC:Steer35Policy','Grid and finite-time policy must match the original1050 batch.');
Dsource=load(fullfile(root,'data','hmc_data.mat'),'D');
coverage=assert_steering_coverage(Dsource.D,3.5);
[V,R]=ndgrid(grid.vc,grid.r);points=[V(:),R(:)];
InputSignature=struct('schema','carsim_fullcar_steer35_150_context_v1', ...
    'extension','append_only_command_3p5deg','assigned_job_ids',[1051 1200], ...
    'legacy_total_cases',1050,'combined_total',1200, ...
    'original_run_signature',Run.Signature,'legacy_batch_signature',Legacy.batch_signature, ...
    'legacy_batch_configuration_sha256',stable_region_sha256(legacyFile), ...
    'extension_configuration',ExtensionConfig,'reference_base_sha256',referenceSHA, ...
    'reference_helper_sha256',helperSHA,'kernel_source_sha256',kernelSourceSHA, ...
    'native_binary_sha256',nativeSHA,'region_binary_sha256',kernelProof.mex_sha256, ...
    'preparer_sha256',stable_region_sha256([mfilename('fullpath') '.m']), ...
    'reference_mode','model_actual_front_trim','grid',grid,'policy',policy, ...
    'steering_coverage',coverage,'source_result_available',false);
batchSignature=text_sha256(jsonencode(InputSignature));
Cases=enumerate_extension(ExtensionConfig);assert(numel(Cases)==150);
if nargin<1||isempty(jobIDs),jobIDs=1051:1200;end
assert(isnumeric(jobIDs)&&isvector(jobIDs)&&all(isfinite(jobIDs))&& ...
    all(jobIDs==fix(jobIDs))&&all(jobIDs>=1051&jobIDs<=1200)&& ...
    numel(unique(jobIDs))==numel(jobIDs),'HMC:Steer35IDs','Use unique global IDs1051..1200.');
jobIDs=jobIDs(:)';
for folder={base,contextDir,errorDir}
    if ~isfolder(folder{1}),mkdir(folder{1});end
end
configurationFile=fullfile(base,'batch_configuration.mat');
if isfile(configurationFile)
    old=load(configurationFile,'Batch');
    assert(strcmp(old.Batch.batch_signature,batchSignature)&& ...
        isequaln(old.Batch.InputSignature,InputSignature), ...
        'HMC:Steer35BatchSignature','Existing extension inputs differ; use a new versioned output folder.');
end
expectedHashes=repmat({''},150,1);indexFile=fullfile(base,'case_index.json');
if isfile(indexFile)
    oldIndex=jsondecode(fileread(indexFile));
    assert(strcmp(oldIndex.batch_signature,batchSignature), ...
        'HMC:Steer35IndexSignature','Saved extension index belongs to different inputs.');
    for k=1:numel(oldIndex.cases)
        oldCase=oldIndex.cases(k);slot=oldCase.job_id-1050;
        assert(slot>=1&&slot<=150&&slot==fix(slot),'HMC:Steer35IndexID','Invalid saved global ID.');
        if strcmp(oldCase.status,'prepared'),expectedHashes{slot}=oldCase.context_sha256;end
    end
end
for slot=1:150
    J=Cases(slot);file=fullfile(contextDir,sprintf('case_%04d.mat',J.job_id));
    if isfile(file)
        saved=load(file,'Context');C=saved.Context;
        assert_valid_context(C,J,batchSignature);
        sha=stable_region_sha256(file);
        assert(isempty(expectedHashes{slot})||strcmp(expectedHashes{slot},sha), ...
            'HMC:Steer35ContextChecksum','Previously indexed context bytes changed: %s',file);
        Cases(slot)=ready_case(J,C,sha);
    else
        failureFile=fullfile(errorDir,sprintf('case_%04d.json',J.job_id));
        if isfile(failureFile)
            failure=jsondecode(fileread(failureFile));
            if strcmp(failure.batch_signature,batchSignature)
                Cases(slot).status='failed';Cases(slot).error=failure.message;
            end
        end
    end
end
Batch=struct('schema','carsim_fullcar_steer35_150_batch_v1','model','carsim_fullcar', ...
    'reference_mode','model_actual_front_trim','total_cases',150,'combined_total',1200, ...
    'legacy_total_cases',1050,'assigned_job_ids',[1051 1200], ...
    'conditions_per_wear',30,'combined_conditions_per_wear',240, ...
    'batch_signature',batchSignature,'legacy_batch_signature',Legacy.batch_signature, ...
    'InputSignature',InputSignature,'original_configuration',Run.Config, ...
    'extension_configuration',ExtensionConfig,'grid',grid,'policy',policy,'cases',Cases);
atomic_mat(configurationFile,'Batch',Batch);publish_index(base,Cases,Batch,jobIDs);
started=tic;newlyPrepared=0;failed=0;resumed=0;
for id=jobIDs
    slot=id-1050;J=Cases(slot);file=fullfile(contextDir,sprintf('case_%04d.mat',id));
    if strcmp(J.status,'prepared')
        resumed=resumed+1;
        fprintf('CONTEXT extension%d/150 global%d wear%d condition%d/30 resumed\n', ...
            slot,id,J.wear_case_id,J.condition_id);
        continue;
    end
    try
        opts=Run.Config.options;
        opts.data_file=fullfile(root,'data','hmc_data.mat');
        opts.calibration_file=fullfile(root,'calibration_active.json');
        opts.speed_kmh=J.speed_kmh;opts.mu=J.mu;opts.steer_deg=J.command_steer_deg;
        opts.wear=J.wear;opts.dt=.0005;opts.duration=.005;
        opts.make_plot=false;opts.save_results=false;opts.progress_stride=0;
        c=HMC_FullCar_Revised('options',opts);
        assert(c.steer_deg==3.5,'HMC:Steer35Command','The extension command must be3.5deg.');
        [Dused,profile]=hmc_apply_calibration(Dsource.D,c);
        [~,T]=hmc_native_run(Dused,c);
        assert(T.root_found&&T.usable&&numel(T.x)==34&&all(isfinite(T.x)), ...
            'HMC:Steer35Trim','An own valid equilibrium is required.');
        c.duration=10;Q=HMC_FullCar_Revised('configuration',c,c.wear,Dused);
        actualSteer=trim_steering(Dused,c,T);Qref=own_reference(Qbase,J,actualSteer);
        H=reference_handling(points,Qref);Htrim=reference_handling(T.x(1:2)',Qref);
        cfg=policy;cfg.mu=J.mu;cfg.reference=Qref;cfg.trim_inside_reference=logical(Htrim.inside);
        Context=struct('job_id',id,'extension_index',slot,'condition_id',J.condition_id, ...
            'wear_case_id',J.wear_case_id,'speed_kmh',J.speed_kmh,'mu',J.mu, ...
            'wear',J.wear,'command_steer_deg',J.command_steer_deg, ...
            'actual_front_steer_deg',actualSteer,'c',c,'Q',Q,'T',T,'Dused',Dused, ...
            'calibration',profile,'cfg',cfg,'grid',grid, ...
            'reference_mask',reshape(logical(H.inside),size(V)), ...
            'reference_mode','model_actual_front_trim','reference_trim',Htrim, ...
            'main_region_known_empty',~logical(Htrim.inside), ...
            'original_run_signature',Run.Signature,'legacy_batch_signature',Legacy.batch_signature, ...
            'input_signature',InputSignature,'batch_signature',batchSignature, ...
            'source_result_file','','source_result_available',false, ...
            'kernel_source_sha256',kernelSourceSHA,'region_binary_sha256',kernelProof.mex_sha256, ...
            'status','prepared','extension','append_only_command_3p5deg', ...
            'initialization','Original model lift, own trim, zero tire restart');
        assert_valid_context(Context,J,batchSignature);atomic_mat(file,'Context',Context);
        Cases(slot)=ready_case(J,Context,stable_region_sha256(file));newlyPrepared=newlyPrepared+1;
        fprintf('CONTEXT extension%d/150 global%d wear%d condition%d/30 H=%d trim_in_H=%d actual=%g\n', ...
            slot,id,J.wear_case_id,J.condition_id,nnz(Context.reference_mask),cfg.trim_inside_reference,actualSteer);
    catch exception
        failed=failed+1;Cases(slot).status='failed';Cases(slot).error=exception.message;
        failure=struct('job_id',id,'extension_index',slot,'status','failed', ...
            'batch_signature',batchSignature,'identifier',exception.identifier,'message',exception.message);
        atomic_json(fullfile(errorDir,sprintf('case_%04d.json',id)),failure);
        fprintf(2,'CONTEXT FAILED global%d extension%d/150: %s\n',id,slot,exception.message);
    end
    publish_index(base,Cases,Batch,jobIDs);
end
Batch.cases=Cases;atomic_mat(configurationFile,'Batch',Batch);
Report=struct('total',150,'combined_total',1200,'assigned_job_ids',[1051 1200], ...
    'requested',numel(jobIDs),'newly_prepared',newlyPrepared,'resumed',resumed, ...
    'failed_requested',failed,'prepared_total',sum(strcmp({Cases.status},'prepared')), ...
    'reference_mode','model_actual_front_trim','elapsed_seconds',toc(started), ...
    'batch_signature',batchSignature,'legacy_batch_signature',Legacy.batch_signature);
atomic_json(fullfile(base,'preparation_report.json'),Report);
end

function Cases=enumerate_extension(cfg)
assert(isscalar(cfg.steers_deg)&&cfg.steers_deg==3.5);
template=struct('job_id',0,'extension_index',0,'condition_id',0,'wear_case_id',0, ...
    'speed_kmh',0,'mu',0,'command_steer_deg',3.5,'wear',zeros(1,4),'status','pending', ...
    'context_file','','context_sha256','','source_result_file','', ...
    'source_result_available',false,'actual_front_steer_deg',NaN,'reference_nodes',NaN, ...
    'trim_inside_reference',false,'main_region_known_empty',false,'error','');
Cases=repmat(template,150,1);slot=0;
for w=1:size(cfg.wear_cases,1)
    condition=0;
    for speed=cfg.speeds_kmh(:)'
        for mu=cfg.mus(:)'
            slot=slot+1;condition=condition+1;J=template;
            J.job_id=1050+slot;J.extension_index=slot;J.condition_id=condition;J.wear_case_id=w;
            J.speed_kmh=speed;J.mu=mu;J.wear=cfg.wear_cases(w,:);
            J.context_file=sprintf('context/case_%04d.mat',J.job_id);Cases(slot)=J;
        end
    end
end
end

function out=ready_case(J,C,sha)
out=J;out.status='prepared';out.context_sha256=sha;out.error='';
out.actual_front_steer_deg=C.actual_front_steer_deg;out.reference_nodes=nnz(C.reference_mask);
out.trim_inside_reference=C.cfg.trim_inside_reference;out.main_region_known_empty=C.main_region_known_empty;
end

function assert_valid_context(C,J,signature)
assert(strcmp(C.batch_signature,signature),'HMC:Steer35ContextSignature','Extension inputs differ.');
assert(C.job_id==J.job_id&&C.extension_index==J.extension_index&& ...
    C.job_id==1050+C.extension_index&&C.condition_id==J.condition_id&& ...
    C.wear_case_id==J.wear_case_id&&C.speed_kmh==J.speed_kmh&&C.mu==J.mu&& ...
    C.command_steer_deg==3.5&&isequal(C.wear,J.wear), ...
    'HMC:Steer35ContextCase','Extension condition mapping differs.');
assert(isempty(C.source_result_file)&&~C.source_result_available, ...
    'HMC:Steer35SourceResult','No original result trajectory exists for a new extension ID.');
assert(C.T.root_found&&C.T.usable&&numel(C.T.x)==34&&all(isfinite(C.T.x)), ...
    'HMC:Steer35ContextTrim','Invalid saved own equilibrium.');
assert(isequal(size(C.reference_mask),[61 121])&& ...
    strcmp(C.reference_mode,'model_actual_front_trim')&& ...
    C.main_region_known_empty==~C.cfg.trim_inside_reference, ...
    'HMC:Steer35ContextReference','Invalid saved H policy or grid.');
end

function degrees=trim_steering(D,c,T)
h=hmc_mex('new',D,c);cleanup=onCleanup(@()hmc_mex('delete',h)); %#ok<NASGU>
[~,O]=hmc_mex('rhs',h,T.x);degrees=mean(O.delta(1:2))*180/pi;
assert(isfinite(degrees),'HMC:Steer35ActualSteering','Actual trim steering must be finite.');
end

function Q=own_reference(base,J,degrees)
M=[1.5662 1.0278 -.17125 -1.1357 -1.0544 -25.85 2.1377 2 8.675e-4 7.046e-5 -.007239 -7.442e-5; ...
   1.5457 1.0286 -.16688 -1.439 -1.6063 -26 1.9164 2 1.82e-3 -7.193e-4 1.802e-3 -.011497; ...
   1.5057 1.0067 -.2019 -2 -2 -27.27 1.5963 2 2.3735e-3 2.38e-4 .01487 -.011025];
Q=base;Q.ux=J.speed_kmh/3.6;Q.mu=J.mu;Q.wear=J.wear;
Q.mu_scale_x=J.mu/.9;Q.mu_scale_y=J.mu/.9;
indices=1+(J.wear(:)>=12.5)+(J.wear(:)>=37.5);Q.MFy=M(indices,:);
Q.delta=[degrees;degrees;0;0]*pi/180;Q.cdelta=cos(Q.delta);Q.sdelta=sin(Q.delta);
Q.reference_mode='model_actual_front_trim';
Q.reference_definition='External static MF H at own model mean front trim steer; never vehicle dynamics';
end

function coverage=assert_steering_coverage(D,degrees)
v=D.vehicle;gear=degrees*v.steering_ratio*pi/180*v.gear_rot_ratio;
names={'FL','FR'};bounds=zeros(2,2);
for k=1:2
    x=v.tables.steering.(names{k}).x_rad(:);bounds(k,:)=[min(x) max(x)];
    assert(isfinite(gear)&&gear>=bounds(k,1)&&gear<=bounds(k,2), ...
        'HMC:Steer35TableRange','3.5deg would be clipped by the original steering table.');
end
coverage=struct('command_deg',degrees,'gear_command_rad',gear,'front_table_bounds_rad',bounds, ...
    'command_inside_both_tables',true,'steering_table_clipped',false);
end

function publish_index(base,Cases,Batch,requested)
report=struct('schema','carsim_fullcar_steer35_150_context_index_v1','total',150, ...
    'combined_total',1200,'legacy_total_cases',1050,'assigned_job_ids',[1051 1200], ...
    'prepared',sum(strcmp({Cases.status},'prepared')), ...
    'failed',sum(strcmp({Cases.status},'failed')),'requested_ids',requested, ...
    'reference_mode','model_actual_front_trim','batch_signature',Batch.batch_signature, ...
    'legacy_batch_signature',Batch.legacy_batch_signature,'cases',Cases);
atomic_json(fullfile(base,'case_index.json'),report);
atomic_json(fullfile(base,'preparation_progress.json'),rmfield(report,'cases'));
file=fullfile(base,'case_index.csv');tmp=[file '.tmp'];fid=fopen(tmp,'w');assert(fid>=0);
fprintf(fid,'extension_index,job_id,extension_progress,wear_case_id,condition_id,condition_progress,speed_kmh,mu,command_steer_deg,wear_FL,wear_FR,wear_RL,wear_RR,status,actual_front_steer_deg,reference_nodes,trim_inside_reference,main_region_known_empty,source_result_available,source_result_file,context_file,context_sha256\n');
for k=1:numel(Cases)
    J=Cases(k);
    fprintf(fid,'%d,%d,%d/150,%d,%d,%d/30,%g,%g,%g,%g,%g,%g,%g,%s,%.17g,%g,%d,%d,%d,%s,%s,%s\n', ...
        J.extension_index,J.job_id,J.extension_index,J.wear_case_id,J.condition_id,J.condition_id, ...
        J.speed_kmh,J.mu,J.command_steer_deg,J.wear,J.status,J.actual_front_steer_deg, ...
        J.reference_nodes,J.trim_inside_reference,J.main_region_known_empty,J.source_result_available, ...
        J.source_result_file,J.context_file,J.context_sha256);
end
fclose(fid);[ok,msg]=movefile(tmp,file,'f');assert(ok,msg);
end

function assert_original_sources(root,S)
assert(strcmp(stable_region_sha256(fullfile(root,'HMC_FullCar_Revised.m')),S.model_sha256), ...
    'HMC:Steer35Model','Original model source changed.');
assert(strcmp(stable_region_sha256(fullfile(root,'data','hmc_data.mat')),S.data_sha256), ...
    'HMC:Steer35Data','Original physical data changed.');
names={'hmc_apply_calibration.m','hmc_native_run.m','hmc_mex.cpp','hmc_native.cpp','tire_fields.inc','active_calibration_file'};
for k=1:numel(names)
    idx=find(strcmp({S.dependencies.name},names{k}));assert(isscalar(idx),'Missing original dependency.');
    name=names{k};if strcmp(name,'active_calibration_file'),name='calibration_active.json';end
    assert(strcmp(stable_region_sha256(fullfile(root,name)),S.dependencies(idx).sha256), ...
        'HMC:Steer35Dependency','Original dependency changed: %s',name);
end
end

function value=text_sha256(text)
if exist('OCTAVE_VERSION','builtin')~=0,value=hash('sha256',text);return;end
digest=javaMethod('getInstance','java.security.MessageDigest','SHA-256');
digest.update(typecast(uint8(unicode2native(text,'UTF-8')),'int8'));
value=lower(reshape(dec2hex(mod(double(digest.digest()),256),2).',1,[]));
end

function atomic_mat(file,name,value)
payload=struct();payload.(name)=value;tmp=[file '.tmp'];save(tmp,'-struct','payload','-v7');
[ok,msg]=movefile(tmp,file,'f');assert(ok,msg);
end

function atomic_json(file,value)
tmp=[file '.tmp'];fid=fopen(tmp,'w');assert(fid>=0);
fprintf(fid,'%s\n',jsonencode(value));fclose(fid);
[ok,msg]=movefile(tmp,file,'f');assert(ok,msg);
end
