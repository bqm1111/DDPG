function [ image ] = imageModel( image,gimbal,T )
%UNTITLED15 Summary of this function goes here
%   Detailed explanation goes here
u = image.u;
w = image.w;
W_ae = gimbal.Wae;
W_ad = gimbal.Wad;
W_ar = gimbal.War;
pan_en = gimbal.pan.angle;
tilt_en = gimbal.tilt.angle;
distance = image.distance;
V = image.velocity;
hfov = image.hfov;

fov = hfov/180*pi;
f = 640/2/tan(fov/2);
R = [cos(tilt_en)*cos(pan_en) cos(tilt_en)*sin(pan_en) -sin(tilt_en);
     - sin(pan_en) cos(pan_en) 0;
     sin(tilt_en)*cos(pan_en) sin(tilt_en)*sin(pan_en) cos(tilt_en)];
Vc = R*[V;0;0];
Vz = Vc(1,1);
Vx = Vc(2,1);
Vy = Vc(3,1);

Wrot = W_ar;

image.du = -f/distance*Vx+u/distance*Vz+u*w/f*W_ae-...
    (f^2+u^2)/f*W_ad+w*W_ar;
image.dw = -f/distance*Vy+w/distance*Vz+(f^2+w^2)/f*W_ae-...
    u*w/f*W_ad-u*W_ar;
image.u = u + image.du * T;
image.w = w + image.dw * T;
image.f = f;
image.Vx = Vx;
image.Vy = Vy;
image.Vz = Vz;
image.Wrot = Wrot;
end

