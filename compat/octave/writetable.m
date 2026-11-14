function writetable(T, destination)
%WRITETABLE CSV export for the scalar columns used by the HMC manifest.
% Octave/Tablicious adapter. Add this folder only when using GNU Octave.
% Model equations and numerical integration do not use this adapter.
assert(nargin==2 && istable(T),'HMC:CSVInput', ...
    'Expected a table and a CSV destination.');
names=T.Properties.VariableNames;
columns=cell(size(names));
for j=1:numel(names), columns{j}=T.(names{j}); end
folder=fileparts(destination);
if isempty(folder), folder=pwd; end
temporary=[tempname(folder) '.tmp'];
cleanup=onCleanup(@()remove_temporary(temporary)); %#ok<NASGU>
[fid,message]=fopen(temporary,'w','n','UTF-8');
assert(fid>=0,'HMC:CSVWrite','%s',message);
closeFile=onCleanup(@()fclose(fid));
fprintf(fid,'%s\n',strjoin(cellfun(@quote_csv,names,'UniformOutput',false),','));
for i=1:height(T)
    cells=cell(1,numel(names));
    for j=1:numel(names)
        column=columns{j};
        if iscell(column), value=column{i}; else, value=column(i,:); end
        if islogical(value) && isscalar(value)
            if value, cells{j}='true'; else, cells{j}='false'; end
        elseif isnumeric(value) && isscalar(value)
            cells{j}=sprintf('%.17g',value);
        elseif ischar(value) || isa(value,'string')
            cells{j}=quote_csv(char(value));
        else
            error('HMC:CSVType','Unsupported column type: %s.',names{j});
        end
    end
    fprintf(fid,'%s\n',strjoin(cells,','));
end
clear closeFile;
[ok,message]=movefile(temporary,destination,'f');
assert(ok,'HMC:CSVWrite','%s',message);
end

function out=quote_csv(text)
out=['"' strrep(text,'"','""') '"'];
end

function remove_temporary(path)
if isfile(path), delete(path); end
end
