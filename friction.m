function [ gimbal ] = friction( gimbal )
%UNTITLED8 Summary of this function goes here
%   Detailed explanation goes here
%% Mass center parameter
el = gimbal.tilt.angle;
az = gimbal.pan.angle;
roll = gimbal.roll;
pitch = gimbal.pitch;
yaw = gimbal.yaw;
% Pan parameter
az_p = -90/180*pi;
el_p = 0/180*pi;
M_pan = 0.545;
D_pan = 0.03756;
% Tilt parameter
az_t = 180/180*pi;
el_t = 0/180*pi;
M_tilt = 1.014;
D_tilt = 0.01223;

g = 9.81;
a = 0;
%% Momen
Rc2b = [cos(az)*cos(el) -sin(az) cos(az)*sin(el);
        sin(az)*cos(el) cos(az)  sin(az)*sin(el);
        -sin(el)        0        cos(el)];
Rb2i = [cos(pitch)*cos(yaw) sin(roll)*sin(pitch)*cos(yaw)-cos(roll)*sin(yaw) cos(roll)*sin(pitch)*cos(yaw)+sin(roll)*sin(yaw);
        cos(pitch)*sin(yaw) sin(roll)*sin(pitch)*sin(yaw)+cos(roll)*cos(yaw) cos(roll)*sin(pitch)*sin(yaw)-sin(roll)*cos(yaw);
        -sin(pitch)         sin(roll)*cos(pitch)                             cos(roll)*cos(pitch)];

G = [0;0;g];
A = [-a*cos(pitch);0;a*sin(pitch)];

% Gia t?c phân ra tr?c pan Tilt
P_tilt = Rc2b'*Rb2i'*(G+A)*M_tilt;

R_tilt = [cos(az_t)*cos(el_t);sin(az_t)*cos(el_t);-sin(el_t)] *D_tilt;
M1 = [P_tilt(3,1)*R_tilt(2,1)-P_tilt(2,1)*R_tilt(3,1);
      P_tilt(1,1)*R_tilt(3,1)-P_tilt(3,1)*R_tilt(1,1);
      P_tilt(2,1)*R_tilt(1,1)-P_tilt(1,1)*R_tilt(2,1)];

%% ?nh h??ng c?a tr?c Pan
Rp = [cos(az) -sin(az) 0;
      sin(az) cos(az)  0;
      0        0       1];
% Gia t?c phân ra tr?c pan Tilt
P_Pan = Rp'*Rb2i'*(G+A)*M_pan;

% Cánh tay l?c tr?c tilt
% Gi? s? trong tâm c?a Tilt n?m cách tâm c?a h? kho?ng d và có các góc
% chi?u d_pan,d_tilt

% R_Pan = Rp *[cos(az_p)*cos(el_p);sin(az_p)*cos(el_p);-sin(el_p)]* D_Pan;
R_Pan = [cos(az_p)*cos(el_p);sin(az_p)*cos(el_p);-sin(el_p)]* D_pan;
M2 = [P_Pan(3,1)*R_Pan(2,1)-P_Pan(2,1)*R_Pan(3,1);
      P_Pan(1,1)*R_Pan(3,1)-P_Pan(3,1)*R_Pan(1,1);
      P_Pan(2,1)*R_Pan(1,1)-P_Pan(1,1)*R_Pan(2,1)];
Rt = [cos(el)  0 sin(el);
      0        1 0;
      -sin(el) 0 cos(el)];
M = Rt*M1 + [0;0;M2(3,1)];

T_fr_el = M(2,1);
T_fr_az = M(3,1);
gimbal.tilt.t_fr_el = T_fr_el;
gimbal.pan.t_fr_az = T_fr_az;
end

