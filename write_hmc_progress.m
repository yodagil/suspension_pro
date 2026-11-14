function report = write_hmc_progress(Summary,outputDir)
%WRITE_HMC_PROGRESS Publish truthful n/210 progress and a per-case index.
% The manifest lists planned cases; only completed/resumed rows count as
% successful simulations. This function does not run or modify the model.

assert(istable(Summary)&&height(Summary)>0, ...
    'HMC210:ProgressSummary','Summary must be a nonempty table.');
if ~isfolder(outputDir),mkdir(outputDir);end
total=height(Summary);
status=string(Summary.Status);
succeeded=ismember(status,["completed","resumed"]);
failed=status=="failed";
processed=succeeded|failed;
requested=status~="not_requested";
report=struct('schema','HMC_210_PROGRESS_1','total',total, ...
    'requested',sum(requested),'succeeded',sum(succeeded), ...
    'processed',sum(processed),'failed',sum(failed), ...
    'pending',sum(status=="pending"),'not_requested',sum(~requested), ...
    'progress_label',sprintf('%d/%d',sum(succeeded),total), ...
    'processed_label',sprintf('%d/%d',sum(processed),total), ...
    'state','waiting','note','Only completed/resumed cases count as successful.', ...
    'cases',[]);
if all(processed|~requested)
    if any(failed),report.state='finished_with_failures';
    else,report.state='finished';end
elseif any(processed)
    report.state='in_progress';
end

item=struct('label','','job_id',0,'condition_id',0,'wear_case_id',0, ...
    'speed_kmh',0,'road_mu',0,'steer_command_deg',0, ...
    'wear_FL_FR_RL_RR',zeros(1,4),'status','','result_file','', ...
    'error_id','','error_message','');
items=repmat(item,total,1);
lines={ '# HMC simulation results'; ''; ...
    sprintf('Successful simulations: **%s**. Processed: **%s**. Failed: **%d**.', ...
        report.progress_label,report.processed_label,report.failed); ''; ...
    sprintf('State: `%s`. Planned cases: %d; requested in this run: %d.', ...
        report.state,total,report.requested); ''; ...
    ['`pending` means this run has not produced or verified a result for that case. ', ...
     '`completed` and `resumed` count as success; `failed` does not.']; ''; ...
    'These are independent model predictions. No CarSim reference comparison is implied.'; ''; ...
    'Each full trajectory is saved as a MAT file. Read `Result.time_s`, `Result.response`, and `Result.state` in MATLAB.'; ''; ...
    '| Case | Speed (km/h) | Road mu | Steering (deg) | Wear FL/FR/RL/RR | Status | Result |'; ...
    '| --- | ---: | ---: | ---: | --- | --- | --- |'};
for k=1:total
    items(k).label=sprintf('%d/%d',Summary.JobID(k),total);
    items(k).job_id=Summary.JobID(k);
    items(k).condition_id=Summary.ConditionID(k);
    items(k).wear_case_id=Summary.WearCaseID(k);
    items(k).speed_kmh=Summary.Speed_kmh(k);
    items(k).road_mu=Summary.RoadMu(k);
    items(k).steer_command_deg=Summary.SteerCommand_deg(k);
    items(k).wear_FL_FR_RL_RR=[Summary.Wear_FL(k),Summary.Wear_FR(k), ...
        Summary.Wear_RL(k),Summary.Wear_RR(k)];
    items(k).status=char(status(k));
    items(k).error_id=char(string(Summary.ErrorID(k)));
    items(k).error_message=char(string(Summary.ErrorMessage(k)));
    resultLink='-';
    if succeeded(k)
        path=char(string(Summary.ResultFile(k)));
        [~,name,ext]=fileparts(path);
        relative=[name ext];
        if isfile(fullfile(outputDir,relative))
            items(k).result_file=relative;
            resultLink=sprintf('[MAT](%s)',relative);
        end
    end
    lines{end+1,1}=sprintf('| %s | %g | %.3g | %g | %g/%g/%g/%g | %s | %s |', ...
        items(k).label,items(k).speed_kmh,items(k).road_mu, ...
        items(k).steer_command_deg,items(k).wear_FL_FR_RL_RR(1), ...
        items(k).wear_FL_FR_RL_RR(2),items(k).wear_FL_FR_RL_RR(3), ...
        items(k).wear_FL_FR_RL_RR(4), ...
        items(k).status,resultLink); %#ok<AGROW>
end
report.cases=items;
write_atomic(fullfile(outputDir,'progress.json'), ...
    [jsonencode(report,'PrettyPrint',true) newline]);
write_atomic(fullfile(outputDir,'README.md'), ...
    [strjoin(lines,newline) newline]);
end

function write_atomic(destination,contents)
folder=fileparts(destination);
temporary=[tempname(folder) '.tmp'];
cleanup=onCleanup(@()remove_temporary(temporary)); %#ok<NASGU>
[fid,message]=fopen(temporary,'w','n','UTF-8');
assert(fid>=0,'HMC210:ProgressWrite','%s',message);
try
    count=fprintf(fid,'%s',contents);
    assert(count>0,'HMC210:ProgressWrite','Could not write progress report.');
catch ME
    fclose(fid);
    rethrow(ME);
end
assert(fclose(fid)==0,'HMC210:ProgressWrite','Could not close progress report.');
[ok,message]=movefile(temporary,destination,'f');
assert(ok,'HMC210:ProgressWrite','%s',message);
end

function remove_temporary(path)
if isfile(path),delete(path);end
end
