function Summary = run_hmc_210(cfg)
%RUN_HMC_210 Run 210 independent full-car operating conditions per wear case.
%   cfg.model_function = 'HMC_FullCar_Revised';
%   cfg.data_file = fullfile(pwd,'data','hmc_data.mat');
%   Summary = run_hmc_210(cfg);
%
%   Defaults: speeds 50:10:100 km/h, mu [.9 .8 .7 .6 .5], and commanded
%   wheel steer 0:.5:3 deg. 6*5*7=210 operating conditions for ONE wear case.
%   cfg.wear_cases=[0 0 0 0;25 25 0 0;50 50 25 25;0 0 25 25;25 25 50 50]
%   requests 1050 trajectories. Corner order is always FL FR RL RR.
%   This is one physical initial state per operating condition, not an
%   initial-state/phase-plane basin sweep. No reference trace drives the model.
%
%   cfg.case_ids selects global JobIDs (default all); cfg.dry_run=true lists
%   the manifest without simulation. cfg.options passes physical model options.
%   cfg.resume=true reuses results only if the model/data/options signature
%   matches. cfg.overwrite=false protects results with different signatures.
%   Individual failures are logged, and the remaining conditions continue.

if nargin<1 || isempty(cfg), cfg=struct(); end
assert(isstruct(cfg)&&isscalar(cfg),'cfg must be a scalar structure.');
here=fileparts(mfilename('fullpath'));
cfg=default_field(cfg,'model_function','HMC_FullCar_Revised');
cfg=default_field(cfg,'data_file',fullfile(here,'data','hmc_data.mat'));
cfg=default_field(cfg,'output_dir',fullfile(here,'result'));
cfg=default_field(cfg,'speeds_kmh',50:10:100);
cfg=default_field(cfg,'mus',[.9 .8 .7 .6 .5]);
cfg=default_field(cfg,'steers_deg',0:.5:3);
cfg=default_field(cfg,'wear_cases',[0 0 0 0]);
cfg=default_field(cfg,'case_ids',[]);
cfg=default_field(cfg,'options',struct());
cfg=default_field(cfg,'resume',true);
cfg=default_field(cfg,'overwrite',false);
cfg=default_field(cfg,'dry_run',false);
assert(numel(cfg.speeds_kmh)*numel(cfg.mus)*numel(cfg.steers_deg)==210, ...
    'The operating-condition grid must contain exactly 210 combinations.');
assert(numel(unique(cfg.speeds_kmh))==numel(cfg.speeds_kmh)&& ...
    numel(unique(cfg.mus))==numel(cfg.mus)&&numel(unique(cfg.steers_deg))==numel(cfg.steers_deg), ...
    'Operating-condition axes must not contain duplicate values.');
assert(all(isfinite(cfg.speeds_kmh))&&all(cfg.speeds_kmh>0), ...
    'Speeds must be finite and positive.');
assert(all(isfinite(cfg.mus))&&all(cfg.mus>0)&&all(isfinite(cfg.steers_deg)), ...
    'Friction must be positive and steering commands finite.');
assert(isnumeric(cfg.wear_cases)&&~isempty(cfg.wear_cases)&&size(cfg.wear_cases,2)==4&& ...
    all(ismember(cfg.wear_cases(:),[0 25 50])), ...
    'wear_cases must have four columns containing 0, 25, or 50.');
assert(isstruct(cfg.options)&&isscalar(cfg.options),'cfg.options must be a structure.');

nWear=size(cfg.wear_cases,1); nJob=210*nWear;
template=struct('JobID',0,'ConditionID',0,'WearCaseID',0,'Speed_kmh',0, ...
    'RoadMu',0,'SteerCommand_deg',0,'Wear_FL',0,'Wear_FR',0,'Wear_RL',0, ...
    'Wear_RR',0,'Status','pending','Seconds',NaN,'ResultFile','', ...
    'RootResidual',NaN,'ReferenceCompared',false,'ErrorID','','ErrorMessage','');
rows=repmat(template,nJob,1); k=0;
for iw=1:nWear
    conditionID=0;
    for iv=1:numel(cfg.speeds_kmh)
        for im=1:numel(cfg.mus)
            for is=1:numel(cfg.steers_deg)
                k=k+1; conditionID=conditionID+1;
                rows(k).JobID=k; rows(k).ConditionID=conditionID; rows(k).WearCaseID=iw;
                rows(k).Speed_kmh=cfg.speeds_kmh(iv); rows(k).RoadMu=cfg.mus(im);
                rows(k).SteerCommand_deg=cfg.steers_deg(is);
                rows(k).Wear_FL=cfg.wear_cases(iw,1); rows(k).Wear_FR=cfg.wear_cases(iw,2);
                rows(k).Wear_RL=cfg.wear_cases(iw,3); rows(k).Wear_RR=cfg.wear_cases(iw,4);
            end
        end
    end
end
selected=cfg.case_ids;
if isempty(selected), selected=1:nJob; end
assert(isnumeric(selected)&&isvector(selected)&&all(isfinite(selected))&& ...
    all(selected==fix(selected))&&all(selected>=1)&all(selected<=nJob), ...
    'case_ids must contain valid integer JobIDs.');
selected=unique(selected(:)','stable');
for k=setdiff(1:nJob,selected), rows(k).Status='not_requested'; end
Summary=struct2table(rows);
fprintf('Design: 210 conditions x %d wear case(s) = %d trajectories; selected %d.\n', ...
    nWear,nJob,numel(selected));
fprintf('One initial vy/r pair per condition; no phase-plane sweep.\n');
if cfg.dry_run, return; end

modelPath=which(char(cfg.model_function));
assert(~isempty(modelPath),'Cannot find model function %s.',char(cfg.model_function));
assert(exist(cfg.data_file,'file')==2,'Cannot find input data MAT: %s',cfg.data_file);
if exist(cfg.output_dir,'dir')~=7, mkdir(cfg.output_dir); end
ModelSHA256=file_sha256(modelPath); DataSHA256=file_sha256(cfg.data_file);
core=cfg.options;
core=remove_fields(core,{'reference_file','reference_files','save_results','make_plot', ...
    'output_dir','figure_visible','progress_stride','data_file','speed_kmh', ...
    'mu','steer_deg','wheelSteer_deg','wear'});
dependencies=dependency_fingerprints(modelPath,core);
Signature=struct('schema','HMC_210_RUN_2','model_function',char(cfg.model_function), ...
    'model_sha256',ModelSHA256,'data_sha256',DataSHA256,'options_json',jsonencode(core), ...
    'dependencies',dependencies);
Config=cfg; %#ok<NASGU>
save(fullfile(cfg.output_dir,'run_configuration.mat'),'Config','Signature','-v7');
writetable(Summary,fullfile(cfg.output_dir,'manifest_210.csv'));
update_progress(Summary,cfg.output_dir);

for ii=1:numel(selected)
    k=selected(ii); spec=rows(k);
    name=sprintf('case_%04d_v%g_mu%.3f_delta%.3f_w%02d.mat', ...
        spec.JobID,spec.Speed_kmh,spec.RoadMu,spec.SteerCommand_deg,spec.WearCaseID);
    path=fullfile(cfg.output_dir,name); rows(k).ResultFile=path;
    CaseSpec=rmfield(spec,{'Status','Seconds','ResultFile','RootResidual', ...
        'ReferenceCompared','ErrorID','ErrorMessage'});
    ticCase=tic;
    fprintf('[%d/%d] job %d: %g km/h, mu %.3g, command %.3g deg, wear [%g %g %g %g]\n', ...
        ii,numel(selected),k,spec.Speed_kmh,spec.RoadMu,spec.SteerCommand_deg, ...
        spec.Wear_FL,spec.Wear_FR,spec.Wear_RL,spec.Wear_RR);
    try
        reuse=false;
        if exist(path,'file')==2
            try
                old=load(path,'Signature','CaseSpec','Result');
                same=isfield(old,'Signature')&&isequaln(old.Signature,Signature)&& ...
                    isfield(old,'CaseSpec')&&isequaln(old.CaseSpec,CaseSpec)&&isfield(old,'Result');
                if cfg.resume&&same
                    validate_result(old.Result,CaseSpec,core);
                    Result=old.Result;reuse=true;
                end
            catch oldError
                if ~cfg.overwrite,rethrow(oldError);end
            end
            if ~reuse&&~cfg.overwrite
                error('HMC210:ExistingResult', ...
                    'Existing output cannot be reused. Select a new output_dir or set overwrite=true: %s',path);
            end
        end
        if reuse
            rows(k).Status='resumed';
        else
            opts=core; opts.data_file=cfg.data_file; opts.output_dir=cfg.output_dir;
            opts.speed_kmh=spec.Speed_kmh; opts.mu=spec.RoadMu; opts.steer_deg=spec.SteerCommand_deg;
            opts.wear=[spec.Wear_FL spec.Wear_FR spec.Wear_RL spec.Wear_RR];
            opts.save_results=false; opts.make_plot=false; opts.figure_visible='off';
            opts.reference_file=''; opts.reference_files={}; opts.progress_stride=0;
            Result=feval(char(cfg.model_function),'run',opts);
            validate_result(Result,CaseSpec,core);
            CompletedHostTime=datestr(now,'yyyy-mm-ddTHH:MM:SS'); %#ok<NASGU>
            % This timestamp uses the MATLAB host clock; it has no asserted offset.
            partial=[tempname(cfg.output_dir) '.mat'];
            save(partial,'Result','CaseSpec','Signature','CompletedHostTime','-v7');
            [ok,msg]=movefile(partial,path,'f');
            assert(ok,'HMC210:SaveFailed','Cannot finalize result: %s',msg);
            rows(k).Status='completed';
        end
        if isfield(Result,'normal_state_check')&&isfield(Result.normal_state_check,'residual_norm')
            rows(k).RootResidual=Result.normal_state_check.residual_norm;
        end
        rows(k).ReferenceCompared=isfield(Result,'reference')&&isfield(Result,'metrics');
    catch ME
        rows(k).Status='failed'; rows(k).ErrorID=ME.identifier; rows(k).ErrorMessage=ME.message;
        fprintf(2,'FAILED job %d: %s\n',k,ME.message);
    end
    rows(k).Seconds=toc(ticCase);
    Summary=struct2table(rows);
    writetable(Summary,fullfile(cfg.output_dir,'run_status.csv'));
    save(fullfile(cfg.output_dir,'run_status.mat'),'Summary','Signature','-v7');
    update_progress(Summary,cfg.output_dir);
end
fprintf('Finished: %d completed, %d resumed, %d failed, %d not requested.\n', ...
    sum(strcmp(Summary.Status,'completed')),sum(strcmp(Summary.Status,'resumed')), ...
    sum(strcmp(Summary.Status,'failed')),sum(strcmp(Summary.Status,'not_requested')));
end

function update_progress(Summary,outputDir)
% A display/export failure must not change a physical simulation's status.
try
    write_hmc_progress(Summary,outputDir);
catch ME
    warning('HMC210:ProgressReport','Progress report could not be updated: %s',ME.message);
end
end

function s=default_field(s,key,value)
if ~isfield(s,key), s.(key)=value; end
end

function s=remove_fields(s,names)
for k=1:numel(names), if isfield(s,names{k}), s=rmfield(s,names{k}); end, end
end

function validate_result(R,spec,opts)
assert(isstruct(R)&&isfield(R,'time_s')&&isfield(R,'response')&&isfield(R,'conditions'), ...
    'HMC210:ResultShape','Model did not return the required result structure.');
t=R.time_s(:); duration=10;
if isfield(opts,'duration'), duration=opts.duration; end
if isfield(opts,'runTime_s'), duration=opts.runTime_s; end
assert(numel(t)>1&&all(isfinite(t))&&all(diff(t)>0)&&abs(t(1))<1e-8&& ...
    abs(t(end)-duration)<1e-7,'HMC210:IncompleteResult','Simulation did not finish the requested interval.');
names={'vy_mps','r_radps','roll_deg','rollrate_degps','pitchrate_degps','vx_kmh'};
for k=1:numel(names)
    assert(isfield(R.response,names{k}),'HMC210:MissingSignal','Missing signal %s.',names{k});
    y=R.response.(names{k});
    assert(isnumeric(y)&&numel(y)==numel(t)&&all(isfinite(y(:))), ...
        'HMC210:InvalidSignal','Invalid or incomplete signal %s.',names{k});
end
c=R.conditions;
assert(abs(c.speed_kmh-spec.Speed_kmh)<1e-9&&abs(c.mu-spec.RoadMu)<1e-9&& ...
    abs(c.steer_command_deg-spec.SteerCommand_deg)<1e-9&& ...
    isequal(reshape(c.wear_FL_FR_RL_RR,1,4),[spec.Wear_FL spec.Wear_FR spec.Wear_RL spec.Wear_RR]), ...
    'HMC210:WrongCondition','Returned trajectory does not match the requested condition.');
end

function out=file_sha256(path)
fid=fopen(path,'rb'); assert(fid>=0,'Cannot open %s for checksum.',path);
cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
bytes=fread(fid,Inf,'*uint8');
if exist('OCTAVE_VERSION','builtin')~=0
    out=hash('sha256',char(bytes(:).'));
else
    md=javaMethod('getInstance','java.security.MessageDigest','SHA-256');
    md.update(typecast(bytes(:),'int8')); raw=md.digest();
    out=lower(reshape(dec2hex(mod(double(raw),256),2).',1,[]));
end
end

function records=dependency_fingerprints(modelPath,opts)
% Include parameters and evaluator dependencies: changing a calibration or
% rebuilding the optional MEX must not silently reuse an older trajectory.
folder=fileparts(modelPath);
names={'hmc_apply_calibration.m','hmc_native_run.m','hmc_mex.cpp', ...
    'hmc_native.cpp','tire_fields.inc'};
records=repmat(struct('name','','sha256',''),0,1);
for k=1:numel(names)
    candidate=fullfile(folder,names{k});
    [~,name,ext]=fileparts(names{k});
    if strcmp(ext,'.m')
        actual=which(name);
        if ~isempty(actual),candidate=actual;end
    end
    if exist(candidate,'file')==2
        records(end+1)=struct('name',names{k},'sha256',file_sha256(candidate)); %#ok<AGROW>
    end
end
calibration=fullfile(folder,'calibration_active.json');
if isfield(opts,'calibration_file'),calibration=char(opts.calibration_file);end
if ~isempty(calibration)&&exist(calibration,'file')==2
    records(end+1)=struct('name','active_calibration_file', ...
        'sha256',file_sha256(calibration)); %#ok<AGROW>
end
mexfile=which('hmc_mex');
if ~isempty(mexfile)&&exist('hmc_mex','file')==3
    records(end+1)=struct('name','available_hmc_mex_binary', ...
        'sha256',file_sha256(mexfile)); %#ok<AGROW>
end
end
