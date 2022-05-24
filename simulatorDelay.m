function [image,control,gimbal,train,state1_o,action1_o] = simulatorDelay(image,control,gimbal,train,environment,state0_o,action0_o)
for i = 1:environment.substeps
    Simulator;
end
T = environment.dt;
N = environment.substeps;
image.uIntegral = image.uIntegral + image.u * N * T;
image.wIntegral = image.wIntegral + image.w * N * T;
state1 = [image.uIntegral;
          image.u;
          (image.u-image.u_1)/N;
          (image.u-2*image.u_1+image.u_2)/N^2;
          image.wIntegral;
          image.w;
          (image.w-image.w_1)/N;
          (image.w-2*image.w_1+image.w_2)/N^2]'; 
image.u_2 = image.u_1;
image.w_2 = image.w_1;
image.u_1 = image.u;
image.w_1 = image.w;

if environment.delay == 0
    state1_o = state1;
else
    train.delay(2:end,:) = train.delay(1:end-1,:);
    train.delay(1,:) = state1;
    image.state = state1;
    state1_o = train.delay(end,:);
    
    u = state0_o(2) + action0_o(2);
    w = state0_o(6) + action0_o(6);
    train.pan.action(2:end,:) = train.pan.action(1:end-1,:);
    train.pan.action(1,1) = (u*w/image.f*gimbal.tilt.gyro - (image.f^2+u^2)/image.f*gimbal.pan.gyro)*N*T;
    delta_u = sum(train.pan.action);
    train.tilt.action(2:end,:) = train.tilt.action(1:end-1,:);
    train.tilt.action(1,1) = ((image.f^2+w^2)/image.f*gimbal.tilt.gyro - u*w/image.f*gimbal.pan.gyro)*N*T;
    delta_w = sum(train.tilt.action);
    action1_o = [action0_o(1,1)+delta_u*N*T;
                 delta_u;
                 (delta_u-action0_o(1,2))/N;
                 (delta_u-action0_o(1,2))/N^2-action0_o(1,3)/N;
                 action0_o(1,5)+delta_w*N*T;
                 delta_w;
                 (delta_w-action0_o(1,6))/N;
                 (delta_w-action0_o(1,6))/N^2-action0_o(1,7)/N]';
end
end