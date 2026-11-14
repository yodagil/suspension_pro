function [Dused,profile] = hmc_apply_calibration(D,c)
%HMC_APPLY_CALIBRATION Apply one declared parameter vector to every condition.
% Original D is preserved. The coefficients are effective reduced-model
% parameters calibrated on the declared healthy-tire reference conditions,
% not certified component values.
% The production model never reads a measured trajectory or measured trim.
Dused=D;
profile=struct('enabled',false,'training_speeds_kmh',[], ...
    'independent_90kph_validation',false,'output_curve_adjustment',false);
if ~isfield(c,'calibration_enabled')||~c.calibration_enabled,return;end
assert(isfield(c,'calibration_file')&&exist(c.calibration_file,'file')==2, ...
    'HMC:CalibrationMissing','The declared calibration JSON file is missing.');
P=jsondecode(fileread(c.calibration_file));
names=cellstr(P.parameter_names);values=double(P.factors(:));
assert(numel(names)==numel(values)&&all(isfinite(values))&&all(values>0), ...
    'HMC:CalibrationValues','Invalid calibration vector.');
p=struct();
for k=1:numel(names),p.(names{k})=values(k);end
required={'Ixx','Iyy','Iz','LKY_front','LKY_rear','Cfy_front','Cfy_rear', ...
    'Cfx_front','Cfx_rear','Karb_front','Karb_rear','speed_KP','speed_KI'};
assert(all(isfield(p,required)),'HMC:CalibrationSchema','Missing effective parameter.');
v=Dused.vehicle;
for key={'Ixx','Iyy','Iz','speed_KP','speed_KI'}
    k=key{1};v.(k)=v.(k)*p.(k);
end
v.Karb_f=v.Karb_f*p.Karb_front;v.Karb_r=v.Karb_r*p.Karb_rear;
v.extra_roll_damping_Nms=0;v.extra_pitch_damping_Nms=0;
if isfield(p,'extra_roll_damping_scaled')
    v.extra_roll_damping_Nms=20000*(p.extra_roll_damping_scaled-1);
end
if isfield(p,'extra_pitch_damping_scaled')
    v.extra_pitch_damping_Nms=20000*(p.extra_pitch_damping_scaled-1);
end
assert(v.extra_roll_damping_Nms>=0&&v.extra_pitch_damping_Nms>=0, ...
    'HMC:CalibrationDamping','Effective added damping must be nonnegative.');
Dused.vehicle=v;
for axle={'front','rear'}
    a=axle{1};targetNames={['new_' a],[a '25'],[a '50']};
    for j=1:numel(targetNames)
        target=targetNames{j};source=target;
        if j==1,source='new';end
        t=D.tires.(source);
        t.LKY=t.LKY*p.(['LKY_' a]);
        t.LATERAL_STIFFNESS=t.LATERAL_STIFFNESS*p.(['Cfy_' a]);
        t.LONGITUDINAL_STIFFNESS=t.LONGITUDINAL_STIFFNESS*p.(['Cfx_' a]);
        Dused.tires.(target)=t;
    end
end
profile=P;profile.enabled=true;profile.profile_file=c.calibration_file;
profile.effective_extra_roll_damping_Nms=v.extra_roll_damping_Nms;
profile.effective_extra_pitch_damping_Nms=v.extra_pitch_damping_Nms;
training=[];
if isfield(P,'source_conditions'),training=reshape(double(P.source_conditions),1,[]);end
profile.training_speeds_kmh=training;
profile.reference_90_used_for_calibration=any(training==90);
profile.note=sprintf(['One common parameter vector, shared across speed/friction/steer/wear. ', ...
    'Healthy-tire reference speeds used in calibration: %s km/h. ', ...
    'Comparisons at those speeds are calibration checks; other operating points ', ...
    'and worn tires require additional numeric reference validation.'],mat2str(training));
end
