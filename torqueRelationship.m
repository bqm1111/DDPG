function [ gimbal ] = torqueRelationship( gimbal )
%UNTITLED7 Summary of this function goes here
%   Detailed explanation goes here
tilt_sp = gimbal.tilt.speed;
tilt_en = gimbal.tilt.angle;
pan_sp  = gimbal.pan.speed;
pan_en  = gimbal.pan.angle;
Tc = 0.03; Ts = 0.0664; ws = 0.05; sigma = 0.0644;
A = [0.546e-3 0.618e-3 0.68e-3];
B = [3.37e-3 3.07e-3 0.935e-3];
W_pi = 0; W_pj = 0; W_pk = 0; d_Wpi = 0; d_Wpj = 0; d_Wpk = 0;
W_be = W_pj*cos(pan_en) - W_pi*sin(pan_en);
W_bn = W_pj*sin(pan_en) + W_pi*cos(pan_en);
W_bk = W_pk + pan_sp;

W_ar = W_bn*cos(tilt_en) - W_bk*sin(tilt_en)+ (rand - 0.5)/180*pi;
W_ad = W_bn*sin(tilt_en) + W_bk*cos(tilt_en)+ (rand - 0.5)/180*pi;
W_ae = W_be + tilt_sp + (rand - 0.5)/180*pi;

Tf_t = Tc * sign(tilt_sp) + (Ts-Tc)*sign(tilt_sp)*exp(-(tilt_sp/ws)^2)+sigma * tilt_sp;
Tf_p = Tc * sign(pan_sp) + (Ts-Tc)*sign(pan_sp)*exp(-(pan_sp/ws)^2)+sigma * pan_sp;

T_d_el = (A(1,3)-A(1,1))*tan(tilt_en)*(W_ad)^2 + (A(1,2)-A(1,1))*W_bn*W_ad/cos(tilt_en)+...
    A(1,2)*(W_bn*pan_sp+d_Wpi*sin(pan_en)-d_Wpj*cos(pan_sp))-Tf_t/2;

T_d_az = W_be*W_bn*(B(1,1)+A(1,1)*(cos(tilt_en))^2+A(1,3)*(sin(tilt_en))^2-B(1,2)-A(1,2))+...
    W_be*W_bk*(A(1,3)-A(1,1))*cos(tilt_en)*sin(tilt_en)+ W_be*pan_sp*(A(1,1)-A(1,3))*sin(tilt_en)*cos(tilt_en)+...
    d_Wpk*(B(1,3)+A(1,1)*(sin(tilt_en))^2+A(1,3)*(cos(tilt_en))^2)+...
    (d_Wpi*cos(pan_en)+d_Wpj*sin(tilt_en))*(A(1,1)-A(1,3))*sin(tilt_en)*cos(tilt_en)+...
    W_bn*tilt_sp*((A(1,1)-A(1,3))*cos(2*tilt_en)-A(1,2))+...
    W_bk*tilt_sp*(A(1,3)-A(1,1))*sin(2*tilt_en)-Tf_p/2;


gimbal.tilt.t_d_el = T_d_el;
gimbal.pan.t_d_az = T_d_az;
gimbal.Wae = W_ae;
gimbal.Wad = W_ad;
gimbal.War = W_ar;
end

