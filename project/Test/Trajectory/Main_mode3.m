%%%%%% Main_mode3 %%%%%%
clear 
clc
%% Constants
Freq = 10;
t_transition = 1; %%From One state to another 
%% Original Position  
P0 = [0.10;0;0.16]; 
theta5_0 = 0;

%% A Position
P_low = [0.14;0;0.10]; %%% Use Joint Space  
P_low_1 = [0.16;0;0.075];
P_low_2 = [0.20;0;0.075]; %%% Use Operational Space]
theta5_low = 42/180*pi;
t_low = 0.5;
%% B Position
P_high = [0.16;0;0.085]; %%% Use Joint Space  
P_high_1 = [0.23;0;0.085];
theta5_high = 132/180*pi;
t_high = 0.5;
%% Set-up of Our Robotic System
L1 = 9.3/100;
L2 = 2.1/100;
L3 = 10.45/100;
L4 = 9.7/100;
L5 = 2.2/100;
L6 = 7/100;

Robot_263C(1) = Link([0  L1  0  0],'modified');
Robot_263C(2) = Link([0  0  L2  pi/2],'modified');
Robot_263C(3) = Link([0  0  L3  0],'modified');
Robot_263C(4) = Link([0  0  L4  0],'modified');
Robot_263C(5) = Link([0  L6  L5  pi/2],'modified');
Robot_263C(6) = Link([0  0  0  0],'modified');
Arm_2 = SerialLink(Robot_263C,'name','Robot_263C');

%% Tranjectory Distribution
T_before = 0;
 %%% Preparation_keep
   t_prepare = 1;
   step_0 = traj_control_Joint(P0,P0,t_prepare,Freq,[theta5_0,theta5_0],T_before);
   T_before = T_before + t_prepare;
 %%% transition work position  
   step_trans_1 = traj_control_Joint(P0,P_low,t_transition,Freq,[theta5_0,theta5_low],T_before);
   step_trans_1(1,:) = [];
   T_before = T_before + t_transition;
 %%% Low Space
   step_L1 = traj_control(P_low,P_low_1,t_low,Freq,theta5_low,T_before);
   step_L1(1,:) = [];
   T_before = T_before + t_low;
   step_L2 = traj_control(P_low_1,P_low_2,t_low,Freq,theta5_low,T_before);
   step_L2(1,:) = [];
   T_before = T_before + t_low;
   step_L3 = traj_control(P_low_2,P_low_1,t_low,Freq,theta5_low,T_before);
   step_L3(1,:) = [];
   T_before = T_before + t_low;
   step_L4 = traj_control(P_low_1,P_low_2,t_low,Freq,theta5_low,T_before);
   step_L4(1,:) = [];
   T_before = T_before + t_low;
   step_L5 = traj_control(P_low_2,P_low_1,t_low,Freq,theta5_low,T_before);
   step_L5(1,:) = [];
   T_before = T_before + t_low;
Step_L = [step_trans_1;step_L1;step_L2;step_L3;step_L4;step_L5];
%%%%%%%====================================================================
   %%% transition work position  
   step_trans_2 = traj_control_Joint(P_low_1,P_high,t_transition,Freq,[theta5_low,theta5_high],T_before);
   step_trans_2(1,:) = [];
   T_before = T_before + t_transition;
 %%% Right Space
   step_H1 = traj_control(P_high,P_high_1,t_high,Freq/2,theta5_high,T_before);
   step_H1(1,:) = [];
   T_before = T_before + t_high;
Step_H = [step_trans_2;step_H1];
%%%%%%%====================================================================
   
 %%% Ending Pos
   step_End = traj_control_Joint(P_high_1,P0,t_transition,Freq,[theta5_high,theta5_low],T_before);
   step_End(1,:) = [];
   T_before = T_before + t_transition;
   step = [
           step_0;
           Step_L;
           Step_H;
           step_End
          ];
      
%% Transform to practical
T = step(:,1).*1000;
theta1_Time = step(:,2);
theta1_Time = round((theta1_Time./pi).*180);
theta2_Time = step(:,3);
theta2_Time = round((theta2_Time./pi).*180);
theta3_Time = step(:,4);
theta3_Time = round((theta3_Time./pi).*180);
theta4_Time = step(:,5);
theta4_Time = round((theta4_Time./pi).*180);
theta5_Time = step(:,6);
theta5_Time = round((theta5_Time./pi).*180);

%% Simulation
figure(3);
pause(5);
theta6_Time = zeros(size(theta1_Time));
for i = 1:1:length(theta1_Time) 
pause(1)
 q = [theta1_Time(i)/180*pi theta2_Time(i)/180*pi theta3_Time(i)/180*pi theta4_Time(i)/180*pi theta5_Time(i)/180*pi theta6_Time(i)/180*pi];
 Arm_2.plot(q);
end

%% Calibration
theta1_Time = theta1_Time + 45;
theta2_Time = 180 - theta2_Time - 58;
theta3_Time = 180 + theta3_Time + 22;
theta4_Time = 90 + theta4_Time - 34 - 10;
% theta_Time = [T,theta1_Time,theta2_Time,theta3_Time,theta4_Time,theta5_Time,theta6_Time];
theta_Time = [];
Theta_Time = [T.' ; theta1_Time.' ; theta2_Time.' ; theta3_Time.' ; theta4_Time.' ; theta5_Time.'];

%% write data to arduino in a .txt file
%to use, open the .txt file and copy paste to arduino
fileID = fopen('matlabToArduino_3.txt','w');

% Theta_Time = zeros(6,2);
col = size(Theta_Time,2);
num = 6 * col;

for j = 1:6
    for i = 1:col
        if i==1 && j==1
            fprintf(fileID,'{{%d,',Theta_Time(j,i));
        elseif i==col && j == 6
            fprintf(fileID,'%d}};',Theta_Time(j,i));
        elseif i==col
            fprintf(fileID,'%d},\n',Theta_Time(j,i));
        elseif i==1
            fprintf(fileID,'{%d,',Theta_Time(j,i));
        else
            fprintf(fileID,'%d,',Theta_Time(j,i));
        end
    end
end

fclose(fileID);

%% Figures for different Joints
figure(1)
plot(T,theta1_Time);hold on;
plot(T,theta2_Time);hold on;
plot(T,theta3_Time);hold on;
plot(T,theta4_Time);hold on;
plot(T,theta5_Time);hold on;
legend('Joint 1','Joint 2','Joint 3','Joint 4','Joint 5');
xlabel('Time(ms)');
ylabel('Angle')
grid on;
title('Desired Joint Angles');

figure(2)
plot(T,step(:,8));hold on;
plot(T,step(:,9));hold on;
plot(T,step(:,10));hold on;
plot(T,step(:,11));hold on;
plot(T,step(:,12));hold on;
legend('Joint 1','Joint 2','Joint 3','Joint 4','Joint 5');
xlabel('Time(ms)');
ylabel('Joint Velocities')
grid on;
title('Desired Joint Velocities');
