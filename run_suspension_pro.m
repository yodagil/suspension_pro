function Summary = run_suspension_pro(cfg)
%RUN_SUSPENSION_PRO Run the supplied HMC model and save outputs in result/.
%   run_suspension_pro
%   Summary = run_suspension_pro(struct('case_ids',1:3));
%   run_suspension_pro(struct('options',struct('engine','mcode')));
%   run_suspension_pro(struct('dry_run',true));
% cfg has the same fields as run_hmc_210, plus optional build_mex (default
% true). Physical/numerical defaults match the uploaded RUN_HMC_PROJECT.m;
% all five wear patterns documented in that package are selected by default.

if nargin<1 || isempty(cfg), cfg=struct(); end
assert(isstruct(cfg)&&isscalar(cfg),'cfg must be a scalar structure.');
root=fileparts(mfilename('fullpath'));
addpath(root);
isOctave=exist('OCTAVE_VERSION','builtin')~=0;
options=struct('engine','auto','variant','revised', ...
    'data_file',fullfile(root,'data','hmc_data.mat'), ...
    'calibration_file',fullfile(root,'calibration_active.json'), ...
    'calibration_enabled',true,'duration',10,'dt',.001, ...
    'vc0_mps',.1,'r0_radps',.35,'I_mA',[950 950 950 950], ...
    'spring_modes',[2 2 2 2]);
if isfield(cfg,'options')
    assert(isstruct(cfg.options)&&isscalar(cfg.options), ...
        'cfg.options must be a scalar structure.');
    names=fieldnames(cfg.options);
    for k=1:numel(names), options.(names{k})=cfg.options.(names{k}); end
end
cfg=default_field(cfg,'model_function','HMC_FullCar_Revised');
cfg=default_field(cfg,'data_file',options.data_file);
cfg=default_field(cfg,'output_dir',fullfile(root,'result'));
cfg=default_field(cfg,'wear_cases',[0 0 0 0;25 25 0 0;50 50 25 25; ...
    0 0 25 25;25 25 50 50]);
cfg=default_field(cfg,'case_ids',[]);
cfg=default_field(cfg,'resume',true);
cfg=default_field(cfg,'overwrite',false);
cfg=default_field(cfg,'dry_run',false);
cfg=default_field(cfg,'build_mex',true);
options.data_file=cfg.data_file;
requestedEngine=lower(char(options.engine));
assert(any(strcmp(requestedEngine,{'auto','mcode','native'})), ...
    'engine must be auto, mcode, or native.');

if ~cfg.dry_run
    % Never select an available MEX implicitly: verify it before use.
    options.engine='mcode';
    if ~strcmp(requestedEngine,'mcode')
        try
            if exist('hmc_mex','file')~=3 && cfg.build_mex && ~isOctave
                compiler=mex.getCompilerConfigurations('C++','Selected');
                if ~isempty(compiler)
                    build_hmc_mex;
                else
                    fprintf(['No supported C++ compiler is selected; ', ...
                        'run mex -setup C++ to enable the optional accelerator.\n']);
                end
            elseif exist('hmc_mex','file')~=3 && cfg.build_mex && isOctave
                fprintf(['GNU Octave: no prebuilt hmc_mex was found. ', ...
                    'Build it with mkoctfile --mex to enable native execution.\n']);
            end
            if exist('hmc_mex','file')==3
                verifyOptions=options;
                verifyOptions.require_mex=true;
                Verification=verify_hmc_runtime(verifyOptions);
                assert(Verification.passed,'HMC:NativeNotVerified', ...
                    'Native implementation equivalence was not verified.');
                if isOctave, Verification.execution_runtime='GNU Octave';
                else, Verification.execution_runtime='MATLAB'; end
                Verification.execution_version=version;
                if exist(cfg.output_dir,'dir')~=7, mkdir(cfg.output_dir); end
                save(fullfile(cfg.output_dir,'runtime_verification.mat'), ...
                    'Verification','-v7');
                options.engine='native';
                fprintf('Using the native MEX after m-code/native verification passed in %s.\n', ...
                    Verification.execution_runtime);
            elseif strcmp(requestedEngine,'native')
                error('HMC:NativeMissing', ...
                    'Native engine requested, but no compiled hmc_mex is available.');
            end
        catch nativeError
            if strcmp(requestedEngine,'native'), rethrow(nativeError); end
            warning('HMC:NativeFallback', ...
                'Native accelerator unavailable or verification failed: %s',nativeError.message);
            options.engine='mcode';
        end
    end
    if strcmp(options.engine,'mcode')
        if isOctave
            fprintf('Using the original m-code evaluator in GNU Octave; no accelerator is used.\n');
        else
            fprintf('Using the pure MATLAB evaluator (mcode); no accelerator is used.\n');
        end
    end
end
cfg.options=options;
cfg=rmfield(cfg,'build_mex');
Summary=run_hmc_210(cfg);
if ~cfg.dry_run
    failures=sum(strcmp(Summary.Status,'failed'));
    assert(failures==0,'HMC:BatchFailed', ...
        '%d requested conditions failed. Inspect run_status.csv in the output directory.',failures);
end
end

function s=default_field(s,key,value)
if ~isfield(s,key), s.(key)=value; end
end
