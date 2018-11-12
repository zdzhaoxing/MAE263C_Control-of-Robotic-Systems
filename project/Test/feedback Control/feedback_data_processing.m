clc
clear all;
close all;

load('mode1_data.mat');

row = size(theta1_Time,1)-2;
col = 5;
real_pos = zeros(row,col);
real_vel = zeros(row,col);

fileID1 = fopen('feedback_mode1_1.txt','r');
fileID2 = fopen('feedback_mode1_2.txt','r');
fileID3 = fopen('feedback_mode1_3.txt','r');
fileID4 = fopen('feedback_mode1_4.txt','r');
formatSpec = '%f';
real_pos(:,1) = fscanf(fileID1,formatSpec);
real_pos(:,2) = fscanf(fileID2,formatSpec);
real_pos(:,3) = fscanf(fileID3,formatSpec);
real_pos(:,4) = fscanf(fileID4,formatSpec);

dt = (T(2)-T(1));
real_vel(2:end,:) = (real_pos(2:end,:) - real_pos(1:end-1,:))/dt*1000;

figure(1)
subplot(4,1,1)
% plot(T,theta1_Time,'r-');hold all;
% plot(T(2:end),real_pos(:,1),'r:');
plot(T(2:end-1),(theta1_Time(2:end-1)-real_pos(:,1)),'r-');hold all;
ylabel('Angle (degree)')
legend('Joint 1 Errors');
title('Mode 1 Joint Angle Errors');
set(gca,'FontSize',15)
grid on;
subplot(4,1,2)
% plot(T,theta2_Time,'b-');hold all;
% plot(T(2:end),real_pos(:,2),'b:');
plot(T(2:end-1),(theta2_Time(2:end-1)-real_pos(:,2)),'b-');hold all;
ylabel('Angle (degree)')
legend('Joint 2 Errors');
set(gca,'FontSize',15)
grid on;
subplot(4,1,3)
% plot(T,theta3_Time,'g-');hold all;
% plot(T(2:end),real_pos(:,3),'g:');
plot(T(2:end-1),(theta3_Time(2:end-1)-real_pos(:,3)),'g-');hold all;
ylabel('Angle (degree)')
legend('Joint 3 Errors');
set(gca,'FontSize',15)
grid on;
subplot(4,1,4)
% plot(T,theta4_Time,'k-');hold all;
% plot(T(2:end),real_pos(:,4),'k:');
plot(T(2:end-1),(theta4_Time(2:end-1)-real_pos(:,4)),'k-');hold all;
legend('Joint 4 Errors');
xlabel('Time(ms)');
ylabel('Angle (degree)')
set(gca,'FontSize',15)
grid on;

% figure(2)
% subplot(4,1,1)
% plot(T,step(:,8),'r-');hold all;
% plot(T(2:end),real_vel(:,1),'r:');
% ylabel('Velocity (degree/s)')
% legend('Joint 1 Desired','Joint 1 Feedback');
% title('Mode 2 Joint Velocities');
% set(gca,'FontSize',15)
% grid on;
% subplot(4,1,2)
% plot(T,step(:,9),'b-');hold all;
% plot(T(2:end),real_vel(:,2),'b:');
% ylabel('Velocity (degree/s)')
% legend('Joint 2 Desired','Joint 2 Feedback');
% set(gca,'FontSize',15)
% grid on;
% subplot(4,1,3)
% plot(T,step(:,10),'g-');hold all;
% plot(T(2:end),real_vel(:,3),'g:');
% ylabel('Velocity (degree/s)')
% legend('Joint 3 Desired','Joint 3 Feedback');
% set(gca,'FontSize',15)
% grid on;
% subplot(4,1,4)
% plot(T,step(:,11),'k-');hold all;
% plot(T(2:end),real_vel(:,4),'k:');
% legend('Joint 4 Desired','Joint 4 Feedback');
% xlabel('Time(ms)');
% ylabel('Velocity (degree/s)')
% set(gca,'FontSize',15)
% grid on;
% 
