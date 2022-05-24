%% Initial parameter
gimbal.pan.angle  = 0; 
gimbal.pan.speed  = 0;
gimbal.pan.current  = 0;
gimbal.pan.gyro  = 0;
gimbal.tilt.angle = 0;
gimbal.tilt.gyro = 0;
gimbal.tilt.current = 0;
gimbal.tilt.speed = 0;
gimbal.roll  = 0; 
gimbal.pitch = 0; 
gimbal.yaw   = 0;
% Control parameter
control.image.L1 = [120 0;0 120];
control.image.L2 = [180 0;0 180];
control.image.z1 = [0;0]; 
control.image.z2 = [0;0];
control.pan.ipanControl = 0;
control.pan.panError = 0;
control.pan.kP = 0.8*180/pi;
control.pan.kI = 20*180/pi;
control.pan.kD = 0*180/pi;
control.tilt.itiltControl = 0;
control.tilt.tiltError = 0;
control.tilt.kP = 0.8*180/pi;
control.tilt.kI = 20*180/pi;
control.tilt.kD = 0*180/pi;
control.controlLim = 1000;
% Initial Camera Model
image.u  = 300*sign(randn); 
image.w  = 200*sign(randn);
image.hfov = 10;
image.f  = 640/2/tan(image.hfov/2*pi/180);
image.distance = 1000;
image.velocity = 0;

[ gimbal ] = torqueRelationship( gimbal );
[ gimbal ] = friction( gimbal );
[ gimbal.pan ] = feedbackIMU( gimbal.Wad,gimbal.pan,environment.dt );
[ gimbal.tilt ] = feedbackIMU( gimbal.Wae,gimbal.tilt,environment.dt );

%% Delay
state0_o = [0,image.u,0,0,0,image.w,0,0];
action0_o = [0 0 0 0 0 0 0 0];
train.pan.action = zeros(environment.delay,1);
train.tilt.action = zeros(environment.delay,1);
train.delay = createDelay(state0_o,environment.delay+1,environment.dt*environment.substeps);
image.uIntegral  = train.delay(1,1);
image.wIntegral  = train.delay(1,5);
image.u_1 = 0;
image.w_1 = 0;
image.u_2 = 0;
image.w_2 = 0;
