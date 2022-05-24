function [gimbal,control] = pidPan( gimbal,control,T )
% function calculator sign after PiD control
%% Input
      % reference : Set point of PID control
      % feedback  : Measured sign feedback
      % kP : Proportinal Gain of PID
      % kI : Integral Gain of PID
      % kD : Derivative Gain of PID
%% Output
      % voltage : Control sign after PID
%% Main
kP = control.pan.kP;
kI = control.pan.kI;
kD = control.pan.kD;
feedback = gimbal.pan.gyro;
reference = gimbal.pan.reference;
ipanControl = control.pan.ipanControl;
panError = control.pan.panError;
error = reference - feedback;
ipanControl = ipanControl + error * T;
voltage = error * kP + ipanControl * kI + kD * (error - panError)/T;
if voltage > 24 || voltage < -24
    voltage = 24 * sign(voltage);
end
panError = error;
control.pan.panError = panError;
control.pan.ipanControl = ipanControl;
gimbal.pan.voltage = voltage;
end

