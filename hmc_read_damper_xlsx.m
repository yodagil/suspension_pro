function damper = hmc_read_damper_xlsx(frontFile, rearFile)
%HMC_READ_DAMPER_XLSX Import the supplied front/rear RS4 damper tables.
% damper = hmc_read_damper_xlsx(frontFile,rearFile)
%
% Reads only RS4_Damper!C4:S4, B5:B22 and C5:S22. The supplied wheel sheet
% contains invalid cached values and is intentionally not used.
% Source convention: positive velocity/force = extension (rebound).
% Output convention: positive velocity/force = compression, as required by
% CarSim CMPRD and FD: F_comp(v,I) = -F_extension(-v,I).
% Keep the 18 current rows by 17 velocity columns; do not round the source
% values, apply lever ratios, or bake in an actuator delay here.
% The simulator should sample and delay its own calculated damper command.

[vf,If,Ff] = read_one(frontFile);
[vr,Ir,Fr] = read_one(rearFile);
if ~isequal(vf,vr) || ~isequal(If,Ir)
    error('HMC:DamperAxes','Front/rear current and velocity axes must match.');
end
% Reverse the axis and its corresponding columns together. This also works
% for any nonsymmetric increasing source axis, without interpolation.
vCompression = -fliplr(vf);
frontCompression = -fliplr(Ff);
rearCompression = -fliplr(Fr);
if any(diff(vCompression)<=0)
    error('HMC:DamperAxes','Converted compression velocity axis must increase.');
end
damper = struct('current_mA',If,'v_mm_s',vCompression, ...
    'front_N',frontCompression,'rear_N',rearCompression, ...
    'current_nominal_mA',950,'compression_sign',1);
damper.note = ['Original RS4_Damper numeric maps, 18x17 each. ', ...
    'Positive velocity/force=compression; F_comp(v,I)=-F_extension(-v,I). ', ...
    'Current in mA, velocity in mm/s, force in N. No extra motion ratio. ', ...
    'Sample/delay are implemented by the simulator, not by this static table.'];
damper.source = struct('front_file',char(frontFile),'rear_file',char(rearFile), ...
    'sheet','RS4_Damper','velocity_range','C4:S4', ...
    'current_range','B5:B22','force_range','C5:S22', ...
    'source_velocity_sign','positive extension', ...
    'output_velocity_sign','positive compression');
end

function [v,I,F] = read_one(file)
file=char(file);
if exist(file,'file')~=2, error('HMC:DamperMissing','Damper file not found: %s',file); end
try
    v=readmatrix(file,'Sheet','RS4_Damper','Range','C4:S4');
    I=readmatrix(file,'Sheet','RS4_Damper','Range','B5:B22');
    F=readmatrix(file,'Sheet','RS4_Damper','Range','C5:S22');
catch err
    error('HMC:DamperRead','Cannot read RS4_Damper numeric ranges in %s: %s',file,err.message);
end
v=reshape(double(v),1,[]); I=reshape(double(I),1,[]); F=double(F);
if numel(v)~=17 || numel(I)~=18 || ~isequal(size(F),[18 17]) ...
        || any(~isfinite(v)) || any(~isfinite(I)) || any(~isfinite(F(:)))
    error('HMC:DamperShape', ...
        'Expected 17 finite velocities, 18 currents and 18x17 finite forces in %s.',file);
end
if any(diff(v)<=0) || any(diff(I)<=0) || ~any(v==0)
    error('HMC:DamperAxes','Velocity/current axes must strictly increase and velocity must include zero.');
end
if any(abs(F(:,v==0))>1e-9)
    error('HMC:DamperZero','Expected zero damping force at zero velocity.');
end
if any(any(F.*repmat(v,size(F,1),1)<-1e-8))
    error('HMC:DamperSign','Source force does not have the documented extension-positive sign.');
end
end
