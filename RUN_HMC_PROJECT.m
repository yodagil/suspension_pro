%% HMC final model-only package: 210 conditions x 5 wear cases = 1050 runs.
% Change only the controls in this section for the usual workflows.
% 'batch'   : run the selected batch conditions only.
% 'smoke'   : independent equilibrium + 5 ms integration at 50/70/90.
% 'preview' : print the condition manifest without any simulation.

HMC_TASK = 'batch';
HMC_BUILD_MEX = false;       % true only after configuring mex -setup C++.
HMC_ENGINE = 'auto';         % 'auto', 'mcode', or 'native'.
HMC_CALIBRATION_ENABLED = true;
HMC_CASE_IDS = [];           % [] = all; e.g. 1:3 = three selected JobIDs.
% Five wear patterns; corner order FL FR RL RR: 0=NEW, 25=25K, 50=50K.
HMC_WEAR_CASES = [0 0 0 0;25 25 0 0;50 50 25 25;0 0 25 25;25 25 50 50];
HMC_RESUME = true;
HMC_OVERWRITE = false;       % Different signatures require a new output folder
                            % or an explicit true value here.

%% Resolve this package without changing the MATLAB working folder.
HMC_ROOT = fileparts(mfilename('fullpath'));
addpath(HMC_ROOT);
assert(any(strcmpi(HMC_TASK,{'batch','smoke','preview'})), ...
    'HMC_TASK must be batch, smoke, or preview.');
HMC_OPTIONS = struct('engine',HMC_ENGINE,'variant','revised', ...
    'data_file',fullfile(HMC_ROOT,'data','hmc_data.mat'), ...
    'calibration_file',fullfile(HMC_ROOT,'calibration_active.json'), ...
    'calibration_enabled',HMC_CALIBRATION_ENABLED,'duration',10,'dt',.001, ...
    'vc0_mps',.1,'r0_radps',.35,'I_mA',[950 950 950 950], ...
    'spring_modes',[2 2 2 2]);
HMC_BATCH_CFG = struct('model_function','HMC_FullCar_Revised', ...
    'data_file',HMC_OPTIONS.data_file,'output_dir',fullfile(HMC_ROOT,'result'), ...
    'wear_cases',HMC_WEAR_CASES,'case_ids',HMC_CASE_IDS,'options',HMC_OPTIONS, ...
    'resume',HMC_RESUME,'overwrite',HMC_OVERWRITE,'dry_run',true);
HMC_Manifest = run_hmc_210(HMC_BATCH_CFG);
disp(HMC_Manifest(1:min(12,height(HMC_Manifest)), ...
    {'JobID','ConditionID','WearCaseID','Speed_kmh','RoadMu','SteerCommand_deg', ...
     'Wear_FL','Wear_FR','Wear_RL','Wear_RR','Status'}));

if ~strcmpi(HMC_TASK,'preview')
    if HMC_BUILD_MEX
        build_hmc_mex;
    end
    if strcmpi(HMC_ENGINE,'auto')&&exist('hmc_mex','file')~=3
        fprintf(['No local MEX accelerator is installed. Using the pure MATLAB evaluator.\n', ...
            'For the optional accelerator, run mex -setup C++ and build_hmc_mex.\n']);
    end
    if strcmpi(HMC_TASK,'smoke')
        HMC_TEST_CONDITIONS = [50 .5 0;70 .6 2.5;90 .6 2.5];
        HMC_TestResults = cell(3,1);
        for HMC_TEST_INDEX=1:3
            HMC_TEST_OPTIONS=HMC_OPTIONS;
            HMC_TEST_OPTIONS.speed_kmh=HMC_TEST_CONDITIONS(HMC_TEST_INDEX,1);
            HMC_TEST_OPTIONS.mu=HMC_TEST_CONDITIONS(HMC_TEST_INDEX,2);
            HMC_TEST_OPTIONS.steer_deg=HMC_TEST_CONDITIONS(HMC_TEST_INDEX,3);
            HMC_TEST_OPTIONS.progress_stride=0;
            HMC_TestResults{HMC_TEST_INDEX}=HMC_FullCar_Revised('selftest',HMC_TEST_OPTIONS);
        end
        fprintf('Smoke checks completed: own equilibrium and 5 ms integration at three conditions.\n');
        fprintf('A smoke check is not a complete 10 s accuracy comparison.\n');
    end
    if strcmpi(HMC_TASK,'batch')
        HMC_BATCH_CFG.dry_run=false;
        HMC_BatchSummary=run_hmc_210(HMC_BATCH_CFG);
    end
else
    fprintf('Preview only. No simulation or MEX build was performed.\n');
end

% Results remain in HMC_BatchSummary / HMC_TestResults.
% No CarSim reference files or precomputed results are needed for simulation.
% Six outputs: vy_mps, rollrate_degps, r_radps, vx_mps, pitchrate_degps, roll_deg.
% The current common calibration was fitted using 50, 70, and 90 km/h data.
