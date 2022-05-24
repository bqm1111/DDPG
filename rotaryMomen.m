function [ struct ] = rotaryMomen( struct,T )
% Calculator torque after motor system
%% Input
      % voltage : Voltage of Motor
      % speed   : Velocity of Motor
      % current : Current of Motor
%% Output
      % torque  : Torque of Motor
      % current : Current of Motor
%% Motor System
voltage = struct.voltage;
speed   = struct.speed;
current = struct.current;
R = 6.6;
L = 0.9e-3;
Km = 0.131;
Kv = 0.130835;
current = exp(-T*R/L)*current + (1-exp(-T*R/L))/R * (voltage-Kv*speed);
torque  = Km * current;
struct.torque  = torque;
struct.current = current;
end

