function [ gimbal ] = tiltModel( gimbal,T )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
torque = gimbal.tilt.torque;
tilt_en = gimbal.tilt.angle;
t_d_el = gimbal.tilt.t_d_el;
t_fr_el = gimbal.tilt.t_fr_el;
tilt_sp = gimbal.tilt.speed;

A = [0.546e-3 0.618e-3 0.68e-3];
B = [3.37e-3 3.07e-3 0.935e-3];
if  tilt_en <= -20/180*pi
    if (torque+t_d_el+t_fr_el)/A(1,2) < 0
        tilt_sp = 0;
        tilt_en = -20/180*pi;
    else
        tilt_sp = tilt_sp + T*(torque+t_d_el+t_fr_el)/A(1,2);
        tilt_en = tilt_en + T*tilt_sp;
    end
elseif tilt_en <= 10/180*pi
    tilt_sp = tilt_sp + T*(torque+t_d_el+t_fr_el)/A(1,2);
    tilt_en = tilt_en + T*tilt_sp;
else
    if (torque+t_d_el+t_fr_el)/A(1,2) > 0
        tilt_sp = 0;
        tilt_en = 10/180*pi;
    else
        tilt_sp = tilt_sp + T*(torque+t_d_el+t_fr_el)/A(1,2);
        tilt_en = tilt_en + T*tilt_sp;
    end
end
gimbal.tilt.speed = tilt_sp;
gimbal.tilt.angle = tilt_en;
end

