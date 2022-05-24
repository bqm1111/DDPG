%% Calculate voltage for motor
[ gimbal,control ] = pidPan( gimbal,control,environment.dt );
[ gimbal,control ] = pidTilt( gimbal,control,environment.dt );
%% Calculate System Parameter
[ gimbal.pan ] = rotaryMomen( gimbal.pan,environment.dt );
[ gimbal.tilt ] = rotaryMomen( gimbal.tilt,environment.dt );
[ gimbal ] = panModel( gimbal,environment.dt );
[ gimbal ] = tiltModel( gimbal,environment.dt );
[ gimbal ] = torqueRelationship( gimbal );
[ gimbal ] = friction( gimbal );
[ gimbal.pan ] = feedbackIMU( gimbal.Wad,gimbal.pan,environment.dt );
[ gimbal.tilt ] = feedbackIMU( gimbal.Wae,gimbal.tilt,environment.dt );
[ image ] = imageModel( image,gimbal,environment.dt );