function report = write_hmc_progress(Summary,outputDir)
%WRITE_HMC_PROGRESS Publish overall and per-wear progress and a case index.
% The manifest lists planned cases; only completed/resumed rows count as
% successful simulations. This function does not run or modify the model.

assert(istable(Summary)&&height(Summary)>0, ...
    'HMC210:ProgressSummary','Summary must be a nonempty table.');
if ~isfolder(outputDir),mkdir(outputDir);end
total=height(Summary);
% Read table columns once: repeated Tablicious property access is expensive.
jobIDs=Summary.JobID;
conditionIDs=Summary.ConditionID;
wearCaseIDs=Summary.WearCaseID;
speeds=Summary.Speed_kmh;
roadMus=Summary.RoadMu;
steerCommands=Summary.SteerCommand_deg;
wearValues=[Summary.Wear_FL,Summary.Wear_FR,Summary.Wear_RL,Summary.Wear_RR];
status=text_column(Summary.Status,total);
resultPaths=text_column(Summary.ResultFile,total);
errorIDs=text_column(Summary.ErrorID,total);
errorMessages=text_column(Summary.ErrorMessage,total);
succeeded=strcmp(status,'completed')|strcmp(status,'resumed');
failed=strcmp(status,'failed');
processed=succeeded|failed;
requested=~strcmp(status,'not_requested');
wearIDs=unique(wearCaseIDs,'stable');
report=struct('schema','HMC_210_PROGRESS_2','total',total, ...
    'wear_case_count',numel(wearIDs),'conditions_per_wear_case',210, ...
    'requested',sum(requested),'succeeded',sum(succeeded), ...
    'processed',sum(processed),'failed',sum(failed), ...
    'pending',sum(strcmp(status,'pending')),'not_requested',sum(~requested), ...
    'progress_label',sprintf('%d/%d',sum(succeeded),total), ...
    'processed_label',sprintf('%d/%d',sum(processed),total), ...
    'state',progress_state(processed,requested,failed), ...
    'note','Only completed/resumed cases count as successful.', ...
    'wear_cases',[],'cases',[]);

wearTemplate=struct('wear_case_id',0,'wear_FL_FR_RL_RR',zeros(1,4), ...
    'total',0,'requested',0,'succeeded',0,'processed',0,'failed',0, ...
    'pending',0,'not_requested',0,'progress_label','','processed_label','', ...
    'state','waiting');
wearRows=repmat(wearTemplate,numel(wearIDs),1);
wearTotalForRow=zeros(total,1);
for iw=1:numel(wearIDs)
    mask=wearCaseIDs==wearIDs(iw);
    first=find(mask,1);wearTotal=sum(mask);wearTotalForRow(mask)=wearTotal;
    wearRows(iw).wear_case_id=wearIDs(iw);
    wearRows(iw).wear_FL_FR_RL_RR=wearValues(first,:);
    wearRows(iw).total=wearTotal;
    wearRows(iw).requested=sum(requested(mask));
    wearRows(iw).succeeded=sum(succeeded(mask));
    wearRows(iw).processed=sum(processed(mask));
    wearRows(iw).failed=sum(failed(mask));
    wearRows(iw).pending=sum(strcmp(status(mask),'pending'));
    wearRows(iw).not_requested=sum(~requested(mask));
    wearRows(iw).progress_label=sprintf('%d/%d',wearRows(iw).succeeded,wearTotal);
    wearRows(iw).processed_label=sprintf('%d/%d',wearRows(iw).processed,wearTotal);
    wearRows(iw).state=progress_state(processed(mask),requested(mask),failed(mask));
end
report.wear_cases=wearRows;

item=struct('label','','job_id',0,'condition_id',0,'condition_label','', ...
    'wear_case_id',0, ...
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
    '## Progress by wear case'; ''; ...
    '| Wear case | Wear FL/FR/RL/RR | Successful | Processed | Failed | State |'; ...
    '| ---: | --- | ---: | ---: | ---: | --- |'};
for iw=1:numel(wearRows)
    wear=wearRows(iw).wear_FL_FR_RL_RR;
    lines{end+1,1}=sprintf('| %d | %g/%g/%g/%g | %s | %s | %d | %s |', ...
        wearRows(iw).wear_case_id,wear(1),wear(2),wear(3),wear(4), ...
        wearRows(iw).progress_label,wearRows(iw).processed_label, ...
        wearRows(iw).failed,wearRows(iw).state); %#ok<AGROW>
end
lines=[lines;{'';'## All simulation cases';''; ...
    '| Global case | Wear case | Condition within wear | Speed (km/h) | Road mu | Steering (deg) | Wear FL/FR/RL/RR | Status | Result |'; ...
    '| --- | ---: | --- | ---: | ---: | ---: | --- | --- | --- |'}];
for k=1:total
    items(k).label=sprintf('%d/%d',jobIDs(k),total);
    items(k).job_id=jobIDs(k);
    items(k).condition_id=conditionIDs(k);
    items(k).condition_label=sprintf('%d/%d',conditionIDs(k),wearTotalForRow(k));
    items(k).wear_case_id=wearCaseIDs(k);
    items(k).speed_kmh=speeds(k);
    items(k).road_mu=roadMus(k);
    items(k).steer_command_deg=steerCommands(k);
    items(k).wear_FL_FR_RL_RR=wearValues(k,:);
    items(k).status=status{k};
    items(k).error_id=errorIDs{k};
    items(k).error_message=errorMessages{k};
    resultLink='-';
    if succeeded(k)
        path=resultPaths{k};
        [~,name,ext]=fileparts(path);
        relative=[name ext];
        if isfile(fullfile(outputDir,relative))
            items(k).result_file=relative;
            resultLink=sprintf('[MAT](%s)',relative);
        end
    end
    lines{end+1,1}=sprintf('| %s | %d | %s | %g | %.3g | %g | %g/%g/%g/%g | %s | %s |', ...
        items(k).label,items(k).wear_case_id,items(k).condition_label, ...
        items(k).speed_kmh,items(k).road_mu, ...
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

function values=text_column(input,count)
% Normalize MATLAB/table and Octave/tablicious text columns to cellstr.
if iscell(input)
    values=input(:);
elseif ischar(input)
    if isempty(input),values=repmat({''},count,1);
    else,values=cellstr(input);end
else
    values=cellstr(input);
    values=values(:);
end
assert(numel(values)==count&&all(cellfun(@ischar,values)), ...
    'HMC210:ProgressText','Expected one character vector per summary row.');
end

function state=progress_state(processed,requested,failed)
state='waiting';
if ~any(requested)
    state='not_requested';
elseif all(processed|~requested)
    if any(failed),state='finished_with_failures';
    else,state='finished';end
elseif any(processed)
    state='in_progress';
end
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
