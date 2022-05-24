function [ gimbal ] = panModel( gimbal,T )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
torque = gimbal.pan.torque;
tilt_en = gimbal.tilt.angle;
pan_en = gimbal.pan.angle;
t_d_az = gimbal.pan.t_d_az;
t_fr_az = gimbal.pan.t_fr_az;
pan_sp = gimbal.pan.speed;

A = [0.546e-3 0.618e-3 0.68e-3];
B = [3.37e-3 3.07e-3 0.935e-3];
if  pan_en <= -45/180*pi
    if (torque+t_d_az+t_fr_az)/(B(1,3)+A(1,1)*(sin(tilt_en))^2+A(1,3)*(cos(tilt_en))^2) < 0
        pan_sp = 0;
        pan_en = -45/180*pi;
    else
        pan_sp = pan_sp + T * (torque+t_d_az+t_fr_az)/(B(1,3)+A(1,1)*(sin(tilt_en))^2+A(1,3)*(cos(tilt_en))^2);
        pan_en = pan_en + T * pan_sp;
    end
elseif pan_en <= 45/180*pi
    pan_sp = pan_sp + T * (torque+t_d_az+t_fr_az)/(B(1,3)+A(1,1)*(sin(tilt_en))^2+A(1,3)*(cos(tilt_en))^2);
    pan_en = pan_en + T * pan_sp;
else
    if (torque+t_d_az+t_fr_az)/(B(1,3)+A(1,1)*(sin(tilt_en))^2+A(1,3)*(cos(tilt_en))^2) > 0
        pan_sp = 0;
        pan_en = 45/180*pi;
    else
        pan_sp = pan_sp + T * (torque+t_d_az+t_fr_az)/(B(1,3)+A(1,1)*(sin(tilt_en))^2+A(1,3)*(cos(tilt_en))^2);
        pan_en = pan_en + T * pan_sp;
    end
end
gimbal.pan.angle = pan_en;
gimbal.pan.speed = pan_sp;
end

