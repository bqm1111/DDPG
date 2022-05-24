function [gimbal,control ] = sceneSteering( state0_o,image,gimbal,control,environment)
%% Input
       % u   : Pixels of Azimuth
       % w   : Pixels of Elevator
       % f   : Focal length of Camera
       % Wae : Elevator Gyro Velocity
       % Wad : Azimuth Gyro Velocity
       
       % Kp  : Proportinal Gain of PID
       % Ki  : Intergral Gain of PID
       
       % tiltEncoder: Elevator angle estimate by encoder
       % L1 (matrix): Luenberger Observer Pixel matrix (eye matrix)
       % L2 (matrix): Luenberger Observer Noise matrix (eye matrix)
%% Output
       % panGyro    : Reference Azimuth of Gyro
       % tiltGyro   : Reference Elevator of Gyro
       % z1 (matrix): Azimuth and Elevator Pixel estimate
       % z2 (matrix): Azimuth and Elevator Noise estimate 
u  = state0_o(1,2);
w  = state0_o(1,6);
Iu = state0_o(1,1);
Iw = state0_o(1,5);
du = state0_o(1,3);
dw = state0_o(1,7);
Wae = gimbal.tilt.gyro;
Wad = gimbal.pan.gyro;
tiltEncoder = gimbal.tilt.angle;
f = image.f;       
z1 = control.image.z1;
z2 = control.image.z2;
T = environment.dt*environment.substeps;
Kp = control.image.Kp;
Ki = control.image.Ki;
Kd = control.image.Kd;
L1 = control.image.L1;
L2 = control.image.L2;
%% Estimated 
B = [u*w/f,-(f^2+u^2)/f;
    (f^2+w^2)/f,-u*w/f];
eOb = z1 - [u;w];
dz1 = z2 - L1 * eOb + B * [Wae;Wad];
dz2 = - L2 * eOb;
z1 = z1 + dz1 * T;
z2 = z2 + dz2 * T;
% z1 = [0;0]; z2 = [0;0];
%% Scene_Steering
U = -inv(B)*( Kp * [u;w] + Ki * [Iu;Iw] + Kd * [du;dw] + z2);
tiltGyro = U(1,1);
panGyro  = U(2,1);

gimbal.pan.reference = panGyro;
gimbal.tilt.reference = tiltGyro;
control.image.z1 = z1;
control.image.z2 = z2;
end

