function writetable(T, destination)
%WRITETABLE CSV export for scalar HMC columns, encoded one column at a time.
% Octave/Tablicious adapter. Physical model calculations do not use it.
assert(nargin==2 && istable(T),'HMC:CSVInput', ...
    'Expected a table and a CSV destination.');
names=T.Properties.VariableNames;
n=height(T); width=numel(names); cells=cell(n,width);
for j=1:width
    column=T.(names{j});
    if n==0, continue; end
    if isnumeric(column) && isreal(column) && size(column,2)==1
        encoded=sprintf('%.17g\n',column);
        values=strsplit(encoded,'\n','CollapseDelimiters',false);
        values(end)=[];
        cells(:,j)=values(:);
    elseif islogical(column) && size(column,2)==1
        labels={'false';'true'};
        cells(:,j)=labels(double(column(:))+1);
    elseif iscellstr(column)
        cells(:,j)=strcat({'"'},strrep(column(:),'"','""'),{'"'});
    elseif isa(column,'string') && size(column,2)==1
        cells(:,j)=strcat({'"'},strrep(cellstr(column(:)),'"','""'),{'"'});
    else
        % Preserve the existing adapter's handling of mixed scalar cells and
        % character matrices, including padded trailing spaces.
        for i=1:n
            if iscell(column), value=column{i}; else, value=column(i,:); end
            if islogical(value) && isscalar(value)
                if value, cells{i,j}='true'; else, cells{i,j}='false'; end
            elseif isnumeric(value) && isscalar(value)
                cells{i,j}=sprintf('%.17g',value);
            elseif ischar(value) || isa(value,'string')
                cells{i,j}=quote_csv(char(value));
            else
                error('HMC:CSVType','Unsupported column type: %s.',names{j});
            end
        end
    end
end
folder=fileparts(destination);
if isempty(folder), folder=pwd; end
temporary=[tempname(folder) '.tmp'];
cleanup=onCleanup(@()remove_temporary(temporary)); %#ok<NASGU>
[fid,message]=fopen(temporary,'w','n','UTF-8');
assert(fid>=0,'HMC:CSVWrite','%s',message);
closeFile=onCleanup(@()fclose(fid));
fprintf(fid,'%s\n',strjoin(cellfun(@quote_csv,names,'UniformOutput',false),','));
if n>0 && width>0
    rowFormat=[repmat('%s,',1,width-1) '%s\n'];
    ordered=cells.';
    fprintf(fid,rowFormat,ordered{:});
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
