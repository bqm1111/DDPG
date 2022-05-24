%%% DDPG apply for Image Gimbal System (RLPIM)
clc;
close all;
clear all;
run('/home/martin/Desktop/RL/Optimization Toolbox/startup.m');
%% PID parameter is constant
run('DDPG_PID.m');
%%
path = 'Save';
mkdir(path,[num2str(year(date)) '-' num2str(month(date)) '-' num2str(day(date))]);
path1 = [path '/' [num2str(year(date)) '-' num2str(month(date)) '-' num2str(day(date))]];
if episodes < parameter.maxEpi
watch = clock;
mkdir(path1,['train_' num2str(watch(4)) 'h_' num2str(watch(5)) 'm_' num2str(environment.delay) '_TDL']);
path2 = [path1 '/' ['train_' num2str(watch(4)) 'h_' num2str(watch(5)) 'm_' num2str(environment.delay) '_TDL']];
save([path2 '/' 'Data.mat']);
saveas(h ,[path2 '/' 'Reward Graph.jpg']);
saveas(hh,[path2 '/' 'Image Response.jpg']);
end
