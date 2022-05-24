function [ struct ] = feedbackIMU( gyro,struct,T )
%UNTITLED23 Summary of this function goes here
%   Detailed explanation goes here
gyro_real = struct.gyro;
T0 = 0.00125;
gyro_real = exp(-T/T0)*gyro_real + (1-exp(-T/T0)) * gyro;
struct.gyro = gyro_real;
end

