function Report=prepare_region_batch(jobIDs)
%PREPARE_REGION_BATCH Own-model contexts for all1050 finite-time region maps.
% prepare_region_batch() prepares all cases; a numeric vector selects a subset.
% The uniform H reference uses each model's own actual trim steering angle.
% The previous three uploaded-CarSim-common-H comparisons remain separate.
% This computes equilibria and5ms preparation runs, never full region grids.
codeDir=fileparts(mfilename('fullpath'));
stableRoot=fileparts(codeDir);root=fileparts(stableRoot);
base=fullfile(stableRoot,'carsim_fullcar_1050');
contextDir=fullfile(base,'context');errorDir=fullfile(base,'preparation_errors');
addpath(root);addpath(codeDir);
assert(exist('hmc_mex','file')==3&& ...
    strcmp(which('hmc_mex'),fullfile(root,['hmc_mex.' mexext])), ...
    'HMC:BatchNative','Build and load the original hmc_mex in the repository root.');
Run=load(fullfile(root,'result','run_configuration.mat'),'Config','Signature');
assert_original_sources(root,Run.Signature);
assert(isequal(Run.Config.wear_cases,[0 0 0 0;25 25 0 0;50 50 25 25; ...
    0 0 25 25;25 25 50 50]),'HMC:BatchWearOrder','Original wear order changed.');
assert(isequal(Run.Config.speeds_kmh(:)',50:10:100)&& ...
    isequal(Run.Config.mus(:)',[.9 .8 .7 .6 .5])&& ...
    isequal(Run.Config.steers_deg(:)',0:.5:3), ...
    'HMC:BatchConditionOrder','Original operating-condition order changed.');
referenceBase=fullfile(stableRoot,'comparison','reference_core','Qref_0.mat');
S=load(referenceBase,'Qref');Qbase=S.Qref;
kernelSourceSHA=stable_region_sha256(fullfile(codeDir,'hmc_region_mex.cpp'));
kernelProof=jsondecode(fileread(fullfile(stableRoot,'comparison','validation','binary_binding.json')));
assert(strcmp(kernelSourceSHA,kernelProof.cpp_sha256), ...
    'HMC:BatchKernelSource','Region kernel source differs from the verified frozen source.');
grid=struct('vc',(-30:30)/10,'r',(-60:60)/40);
[V,R]=ndgrid(grid.vc,grid.r);points=[V(:),R(:)];
policy=struct('dt',.0005,'observe_dt',.001,'horizons',[5 10], ...
    'terminal_window',1,'policy_g',9.81,'yaw_factor',.85,'ay_factor',1, ...
    'LTR_limit',1,'roll_limit_rad',40*pi/180,'early_reject_final',true, ...
    'window_start_inclusive',true,'main_requires_ay',false,'threads',4, ...
    'trace_stride',0,'terminal_tolerance',[.1 .005 .2 .2 .5 .5 .5]);
InputSignature=struct('schema','carsim_fullcar_1050_context_v1', ...
    'original_run_signature',Run.Signature, ...
    'reference_base_sha256',stable_region_sha256(referenceBase), ...
    'kernel_source_sha256',kernelSourceSHA, ...
    'preparer_sha256',stable_region_sha256([mfilename('fullpath') '.m']), ...
    'reference_mode','model_actual_front_trim','grid',grid,'policy',policy);
batchSignature=text_sha256(jsonencode(InputSignature));
Cases=enumerate_cases(Run.Config);
assert(numel(Cases)==1050);
if nargin<1||isempty(jobIDs),jobIDs=1:1050;end
assert(isnumeric(jobIDs)&&isvector(jobIDs)&&all(isfinite(jobIDs))&& ...
    all(jobIDs==fix(jobIDs))&&all(jobIDs>=1&jobIDs<=1050)&& ...
    numel(unique(jobIDs))==numel(jobIDs),'HMC:BatchIDs','Use unique integer case IDs1..1050.');
jobIDs=jobIDs(:)';
for folder={base,contextDir,errorDir}
    if ~isfolder(folder{1}),mkdir(folder{1});end
end
configurationFile=fullfile(base,'batch_configuration.mat');
if isfile(configurationFile)
    old=load(configurationFile,'Batch');
    assert(strcmp(old.Batch.batch_signature,batchSignature), ...
        'HMC:BatchSignature','Existing batch inputs differ; use a new versioned output folder.');
    assert(isequaln(old.Batch.InputSignature,InputSignature),'HMC:BatchSignature','Input-signature collision.');
end
% Existing contexts are independently checked, not trusted from an old index.
expectedHashes=repmat({''},1050,1);
indexFile=fullfile(base,'case_index.json');
if isfile(indexFile)
    previous=jsondecode(fileread(indexFile));
    assert(strcmp(previous.batch_signature,batchSignature),'HMC:BatchIndexSignature','Saved index belongs to different inputs.');
    for k=1:numel(previous.cases)
        oldCase=previous.cases(k);
        if strcmp(oldCase.status,'prepared'),expectedHashes{oldCase.job_id}=oldCase.context_sha256;end
    end
end
for id=1:1050
    f=fullfile(contextDir,sprintf('case_%04d.mat',id));
    if isfile(f)
        saved=load(f,'Context');C=saved.Context;
        assert_valid_context(C,Cases(id),batchSignature);
        sha=stable_region_sha256(f);
        assert(isempty(expectedHashes{id})||strcmp(expectedHashes{id},sha), ...
            'HMC:BatchContextChecksum','Previously indexed context bytes changed: %s',f);
        Cases(id)=ready_case(Cases(id),C,sha);
    else
        failureFile=fullfile(errorDir,sprintf('case_%04d.json',id));
        if isfile(failureFile)
            previousFailure=jsondecode(fileread(failureFile));
            if strcmp(previousFailure.batch_signature,batchSignature)
                Cases(id).status='failed';Cases(id).error=previousFailure.message;
            end
        end
    end
end
Batch=struct('schema','carsim_fullcar_1050_batch_v1','model','carsim_fullcar', ...
    'reference_mode','model_actual_front_trim','total_cases',1050, ...
    'conditions_per_wear',210,'batch_signature',batchSignature, ...
    'InputSignature',InputSignature,'original_configuration',Run.Config, ...
    'grid',grid,'policy',policy,'cases',Cases);
atomic_mat(configurationFile,'Batch',Batch);
publish_index(base,Cases,batchSignature,jobIDs);
Dsource=load(fullfile(root,'data','hmc_data.mat'),'D');
started=tic;newlyPrepared=0;failed=0;resumed=0;
for id=jobIDs
    J=Cases(id);file=fullfile(contextDir,sprintf('case_%04d.mat',id));
    if strcmp(J.status,'prepared')
        resumed=resumed+1;
        fprintf('CONTEXT %d/1050 (%d/210 wear%d) resumed\n',id,J.condition_id,J.wear_case_id);
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
        [Dused,profile]=hmc_apply_calibration(Dsource.D,c);
        [~,T]=hmc_native_run(Dused,c);
        assert(T.root_found&&T.usable&&all(isfinite(T.x)), ...
            'HMC:BatchTrim','An own valid equilibrium is required.');
        c.duration=10;Q=HMC_FullCar_Revised('configuration',c,c.wear,Dused);
        actualSteer=trim_steering(Dused,c,T);
        Qref=own_reference(Qbase,J,actualSteer);
        H=reference_handling(points,Qref);
        Htrim=reference_handling(T.x(1:2)',Qref);
        cfg=policy;cfg.mu=J.mu;cfg.reference=Qref;
        cfg.trim_inside_reference=logical(Htrim.inside);
        Context=struct('job_id',id,'condition_id',J.condition_id, ...
            'wear_case_id',J.wear_case_id,'speed_kmh',J.speed_kmh, ...
            'mu',J.mu,'wear',J.wear,'command_steer_deg',J.command_steer_deg, ...
            'actual_front_steer_deg',actualSteer,'c',c,'Q',Q,'T',T, ...
            'Dused',Dused,'calibration',profile,'cfg',cfg,'grid',grid, ...
            'reference_mask',reshape(logical(H.inside),size(V)), ...
            'reference_mode','model_actual_front_trim', ...
            'reference_trim',Htrim,'main_region_known_empty',~logical(Htrim.inside), ...
            'original_run_signature',Run.Signature,'input_signature',InputSignature, ...
            'batch_signature',batchSignature,'source_result_file',J.source_result_file, ...
            'kernel_source_sha256',kernelSourceSHA, ...
            'status','prepared','initialization','Original model lift, own trim, zero tire restart');
        assert_valid_context(Context,J,batchSignature);
        atomic_mat(file,'Context',Context);
        Cases(id)=ready_case(J,Context,stable_region_sha256(file));
        newlyPrepared=newlyPrepared+1;
        fprintf('CONTEXT %d/1050 (%d/210 wear%d) H=%d trim_in_H=%d actual=%g\n', ...
            id,J.condition_id,J.wear_case_id,nnz(Context.reference_mask), ...
            cfg.trim_inside_reference,actualSteer);
    catch exception
        failed=failed+1;Cases(id).status='failed';Cases(id).error=exception.message;
        failure=struct('job_id',id,'status','failed','batch_signature',batchSignature, ...
            'identifier',exception.identifier,'message',exception.message);
        atomic_json(fullfile(errorDir,sprintf('case_%04d.json',id)),failure);
        fprintf(2,'CONTEXT FAILED %d/1050: %s\n',id,exception.message);
    end
    publish_index(base,Cases,batchSignature,jobIDs);
end
Batch.cases=Cases;atomic_mat(configurationFile,'Batch',Batch);
Report=struct('total',1050,'requested',numel(jobIDs),'newly_prepared',newlyPrepared, ...
    'resumed',resumed,'failed_requested',failed,'prepared_total',sum(strcmp({Cases.status},'prepared')), ...
    'reference_mode','model_actual_front_trim','elapsed_seconds',toc(started), ...
    'batch_signature',batchSignature);
atomic_json(fullfile(base,'preparation_report.json'),Report);
end

function Cases=enumerate_cases(cfg)
template=struct('job_id',0,'condition_id',0,'wear_case_id',0,'speed_kmh',0, ...
    'mu',0,'command_steer_deg',0,'wear',zeros(1,4),'status','pending', ...
    'context_file','','context_sha256','','source_result_file','', ...
    'actual_front_steer_deg',NaN,'reference_nodes',NaN, ...
    'trim_inside_reference',false,'main_region_known_empty',false,'error','');
Cases=repmat(template,1050,1);id=0;
for w=1:size(cfg.wear_cases,1)
    condition=0;
    for speed=cfg.speeds_kmh(:)'
        for mu=cfg.mus(:)'
            for angle=cfg.steers_deg(:)'
                id=id+1;condition=condition+1;C=template;
                C.job_id=id;C.condition_id=condition;C.wear_case_id=w;
                C.speed_kmh=speed;C.mu=mu;C.command_steer_deg=angle;C.wear=cfg.wear_cases(w,:);
                C.context_file=sprintf('context/case_%04d.mat',id);
                C.source_result_file=sprintf('result/case_%04d_v%g_mu%.3f_delta%.3f_w%02d.mat', ...
                    id,speed,mu,angle,w);Cases(id)=C;
            end
        end
    end
end
end

function out=ready_case(J,C,sha)
out=J;out.status='prepared';out.context_sha256=sha;out.error='';
out.actual_front_steer_deg=C.actual_front_steer_deg;
out.reference_nodes=nnz(C.reference_mask);
out.trim_inside_reference=C.cfg.trim_inside_reference;
out.main_region_known_empty=C.main_region_known_empty;
end

function assert_valid_context(C,J,signature)
assert(strcmp(C.batch_signature,signature),'HMC:BatchContextSignature','Context belongs to different inputs.');
assert(C.job_id==J.job_id&&C.condition_id==J.condition_id&&C.wear_case_id==J.wear_case_id&& ...
    C.speed_kmh==J.speed_kmh&&C.mu==J.mu&&C.command_steer_deg==J.command_steer_deg&& ...
    isequal(C.wear,J.wear),'HMC:BatchContextCase','Context operating condition differs.');
assert(C.T.root_found&&C.T.usable&&all(isfinite(C.T.x))&&numel(C.T.x)==34, ...
    'HMC:BatchContextTrim','Invalid saved own equilibrium.');
assert(isequal(size(C.reference_mask),[61 121])&& ...
    strcmp(C.reference_mode,'model_actual_front_trim')&& ...
    C.main_region_known_empty==~C.cfg.trim_inside_reference, ...
    'HMC:BatchContextReference','Invalid reference policy or grid.');
end

function degrees=trim_steering(D,c,T)
h=hmc_mex('new',D,c);cleanup=onCleanup(@()hmc_mex('delete',h)); %#ok<NASGU>
[~,O]=hmc_mex('rhs',h,T.x);degrees=mean(O.delta(1:2))*180/pi;
assert(isfinite(degrees),'HMC:BatchSteering','Actual trim steering must be finite.');
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

function publish_index(base,Cases,signature,requested)
report=struct('schema','carsim_fullcar_1050_context_index_v1','total',1050, ...
    'prepared',sum(strcmp({Cases.status},'prepared')), ...
    'failed',sum(strcmp({Cases.status},'failed')),'requested_ids',requested, ...
    'reference_mode','model_actual_front_trim','batch_signature',signature,'cases',Cases);
atomic_json(fullfile(base,'case_index.json'),report);
atomic_json(fullfile(base,'preparation_progress.json'),rmfield(report,'cases'));
file=fullfile(base,'case_index.csv');tmp=[file '.tmp'];fid=fopen(tmp,'w');assert(fid>=0);
fprintf(fid,'job_id,global_progress,wear_case_id,condition_id,condition_progress,speed_kmh,mu,command_steer_deg,wear_FL,wear_FR,wear_RL,wear_RR,status,actual_front_steer_deg,reference_nodes,trim_inside_reference,main_region_known_empty,context_file,context_sha256\n');
for k=1:numel(Cases)
    C=Cases(k);
    fprintf(fid,'%d,%d/1050,%d,%d,%d/210,%g,%g,%g,%g,%g,%g,%g,%s,%.17g,%g,%d,%d,%s,%s\n', ...
        C.job_id,C.job_id,C.wear_case_id,C.condition_id,C.condition_id,C.speed_kmh,C.mu, ...
        C.command_steer_deg,C.wear,C.status,C.actual_front_steer_deg,C.reference_nodes, ...
        C.trim_inside_reference,C.main_region_known_empty,C.context_file,C.context_sha256);
end
fclose(fid);[ok,msg]=movefile(tmp,file,'f');assert(ok,msg);
end

function assert_original_sources(root,Signature)
assert(strcmp(stable_region_sha256(fullfile(root,'HMC_FullCar_Revised.m')),Signature.model_sha256), ...
    'HMC:BatchModel','Original1050 model source differs.');
assert(strcmp(stable_region_sha256(fullfile(root,'data','hmc_data.mat')),Signature.data_sha256), ...
    'HMC:BatchData','Original1050 physical data differs.');
names={'hmc_apply_calibration.m','hmc_native_run.m','hmc_mex.cpp','hmc_native.cpp','tire_fields.inc','active_calibration_file'};
for k=1:numel(names)
    i=find(strcmp({Signature.dependencies.name},names{k}));assert(isscalar(i),'Missing source dependency.');
    filename=names{k};if strcmp(filename,'active_calibration_file'),filename='calibration_active.json';end
    assert(strcmp(stable_region_sha256(fullfile(root,filename)),Signature.dependencies(i).sha256), ...
        'HMC:BatchDependency','Original1050 dependency differs: %s',filename);
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
