function ref = read_carsim_reference(source, gravity_mps2)
%READ_CARSIM_REFERENCE Read the 72-channel CarSim export without curve fitting.
% ref = read_carsim_reference(file)
% ref = read_carsim_reference(file, gravity_mps2)
%
% Accepted files:
%   * Original CarSimOut_trim MATLAB timeseries (including native v7.3).
%   * CarSimOut_trim struct with numeric Data and Time fields.
%   * Flat ref/Ref struct made by inspect_carsim.py, with raw_export_data and
%     time_absolute_s (or time_s and recorded info.start_s).
% A corresponding structure can also be supplied directly.
%
% The original 72 EXPORT channels must be present. Units are taken from the
% attached trim_input.par EXPORT list, never inferred from the filename.
% VX_SM/VY_SM are heading-frame velocities; AVX/AVY/AVZ are body-fixed rates.
% Only the recorded first timestamp is subtracted. No amplitude correction,
% smoothing, fitted time shift, steady offset subtraction or transient cut.

if nargin < 2 || isempty(gravity_mps2), gravity_mps2 = 9.80665; end
validateattributes(gravity_mps2, {'numeric'}, {'scalar','real','finite','positive'});
sourceFile = '';
if ischar(source) || (isstring(source) && isscalar(source))
    sourceFile = char(source);
    if exist(sourceFile,'file') ~= 2
        error('HMC:ReferenceMissing','Reference file does not exist: %s',sourceFile);
    end
    S = load(sourceFile); % MATLAB decodes its native timeseries/MCOS object.
elseif isstruct(source) || isa(source,'timeseries')
    S = source;
else
    error('HMC:ReferenceType','Provide a MAT file, timeseries, or reference structure.');
end
info = struct(); previousProvenance = struct();
if isstruct(S) && isfield(S,'info'), info = S.info; end
if isstruct(S) && isfield(S,'CarSimOut_trim')
    v = S.CarSimOut_trim;
elseif isstruct(S) && isfield(S,'ref')
    v = S.ref;
elseif isstruct(S) && isfield(S,'Ref')
    v = S.Ref;
else
    v = S;
end
if isstruct(v) && isfield(v,'info'), info = v.info; end
if isstruct(v) && isfield(v,'provenance'), previousProvenance = v.provenance; end
if isempty(info), info = struct(); end
if ~isstruct(info) || ~isscalar(info)
    error('HMC:ReferenceInfo','Reference info must be a scalar structure.');
end
if isa(v,'timeseries')
    Y = v.Data; t = v.Time;
elseif isstruct(v) && isfield(v,'Data') && isfield(v,'Time')
    Y = v.Data; t = v.Time;
elseif isstruct(v) && isfield(v,'raw_export_data')
    Y = v.raw_export_data;
    if isfield(v,'time_absolute_s')
        t = v.time_absolute_s;
    elseif isfield(v,'time_s')
        t = v.time_s;
        if isfield(info,'start_s') && isnumeric(info.start_s) && ...
                isscalar(info.start_s) && isfinite(info.start_s)
            t = t + double(info.start_s);
        end
    else
        error('HMC:ReferenceTime','Flat reference lacks time_absolute_s or time_s.');
    end
else
    error('HMC:ReferenceFormat', ...
        ['Expected CarSimOut_trim timeseries/Data-Time struct, or flat ref ', ...
         'with raw_export_data. A six-signal-only file cannot verify EXPORT ordering.']);
end
if ~isnumeric(Y) || ~isnumeric(t) || ~isreal(Y) || ~isreal(t)
    error('HMC:ReferenceNumeric','Reference Data and Time must be real numeric arrays.');
end
Y = squeeze(double(Y)); t = double(t(:));
if numel(t) < 2 || ~ismatrix(Y)
    error('HMC:ReferenceShape','Reference needs at least two times and a 2-D data array.');
end
if size(Y,1)==72 && size(Y,2)==numel(t) && size(Y,1)~=numel(t), Y=Y.'; end
if size(Y,1)~=numel(t) || size(Y,2)~=72
    error('HMC:ReferenceShape','Reference must have N-by-72 EXPORT channels; got %d-by-%d for %d times.',size(Y,1),size(Y,2),numel(t));
end
if any(~isfinite(Y(:))) || any(~isfinite(t)) || any(diff(t)<=0)
    error('HMC:ReferenceSamples','All samples must be finite and timestamps strictly increasing.');
end
if isfield(info,'start_s') && isnumeric(info.start_s) && isscalar(info.start_s) ...
        && isfinite(info.start_s) && abs(t(1)-double(info.start_s)) > 1e-6
    error('HMC:ReferenceTimeMetadata','Recorded start_s differs from the first reference timestamp.');
end
if isfield(info,'stop_s') && isnumeric(info.stop_s) && isscalar(info.stop_s) ...
        && isfinite(info.stop_s) && abs(t(end)-double(info.stop_s)) > 1e-6
    error('HMC:ReferenceTimeMetadata','Recorded stop_s differs from the last reference timestamp.');
end

ref = struct();
ref.time_s = t-t(1); ref.time_absolute_s = t;
ref.response = struct('vy_mps',Y(:,4)/3.6,'r_radps',Y(:,14)*pi/180, ...
    'roll_deg',Y(:,9),'pitch_deg',Y(:,10), ...
    'rollrate_degps',Y(:,12),'pitchrate_degps',Y(:,13), ...
    'vx_kmh',Y(:,3),'ay_mps2',Y(:,7)*gravity_mps2,'vx_mps',Y(:,3)/3.6);
ref.info = info;
ref.conditions = struct();
conditionFields = {'speed_kmh','mu','steer_command_deg','steer_actual_deg', ...
    'wear_FL_FR_RL_RR','initial_vc_mps','initial_yaw_radps','spring_mode','cdc_current'};
for k=1:numel(conditionFields)
    key=conditionFields{k};
    if isfield(info,key), ref.conditions.(key)=info.(key); end
end
ref.condition_info_complete = all(isfield(info,{'speed_kmh','mu','steer_command_deg'}));
ref.raw_export_data = Y;
ref.units = struct('raw_velocity','km/h','raw_angles','deg','raw_rates','deg/s', ...
    'raw_acceleration','g','gravity_mps2',gravity_mps2);
ref.frames = struct('velocity','yaw-heading horizontal axes plus global vertical', ...
    'angular_rate','body-fixed p/q/r, not Euler-angle derivatives');
ref.provenance = struct('reader','read_carsim_reference', ...
    'source_file',sourceFile,'source_provenance',previousProvenance, ...
    'sample_count',numel(t),'channel_count',72, ...
    'absolute_start_s',t(1),'absolute_stop_s',t(end), ...
    'time_operation','Subtract recorded first timestamp only', ...
    'amplitude_operation','None','transient_samples_removed',0);
ref.note = ['Conditions come from embedded info. Filenames are not authoritative. ', ...
    'No fitted time shift, rescaling, smoothing or transient removal.'];
end
