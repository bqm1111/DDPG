function [ gimbal,control ] = pidTilt( gimbal,control,T )
%% Input
      % reference : Set point of PID control
      % feedback  : Measured sign feedback
      % kP : Proportinal Gain of PID
      % kI : Integral Gain of PID
      % kD : Derivative Gain of PID
%% Output
      % voltage : Control sign after PID
%% Main
kP = control.tilt.kP;
kI = control.tilt.kI;
kD = control.tilt.kD;
feedback  = gimbal.tilt.gyro;
reference = gimbal.tilt.reference;
itiltControl = control.tilt.itiltControl;
tiltError = control.tilt.tiltError;
error = reference - feedback;
itiltControl = itiltControl + error * T;
voltage = error * kP + itiltControl * kI + kD * (error - tiltError)/T;
if voltage > 24 || voltage < -24
    voltage = 24 * sign(voltage);
end
tiltError = error;
control.tilt.tiltError = tiltError;
control.tilt.itiltControl = itiltControl;
gimbal.tilt.voltage = voltage;
end

