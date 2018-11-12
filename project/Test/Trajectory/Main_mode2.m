%%%%%% Main_mode1 %%%%%%
clear 
clc
%% Constants
Freq = 10;
t_transition = 1; %%From One state to another 
%% Original Position  
P0 = [0.10;0;0.16]; 
theta5_0 = 0;

%% A Position
P_mid = [0.14;0;0.10]; %%% Use Joint Space  
P_mid_1 = [0.16;0;0.075];
P_mid_2 = [0.20;0;0.075]; %%% Use Operational Space]
theta5_mid = 42/180*pi;
t_mid = 0.5;
%% B Position
P_right = [0.18;0.06;0.10]; %%% Use Joint Space  
P_right_1 = [0.20;0.05;0.075];
P_right_2 = [0.22;0.03;0.075]; %%% Use Operational Space]
theta5_right = 84/180*pi;
t_right = 0.5;
%% C Position
P_left = [0.18;0.06;0.10]; %%% Use Joint Space  
P_left_1 = [0.20;-0.05;0.075];
P_left_2 = [0.22;-0.03;0.075]; %%% Use Operational Space]
theta5_left = 0/180*pi;
t_left = 0.5;
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
   step_trans_1 = traj_control_Joint(P0,P_mid,t_transition,Freq,[theta5_0,theta5_mid],T_before);
   step_trans_1(1,:) = [];
   T_before = T_before + t_transition;
 %%% Mid Space
   step_MW1 = traj_control(P_mid,P_mid_1,t_mid,Freq,theta5_mid,T_before);
   step_MW1(1,:) = [];
   T_before = T_before + t_mid;
   step_MW2 = traj_control(P_mid_1,P_mid_2,t_mid,Freq,theta5_mid,T_before);
   step_MW2(1,:) = [];
   T_before = T_before + t_mid;
   step_MW3 = traj_control(P_mid_2,P_mid_1,t_mid,Freq,theta5_mid,T_before);
   step_MW3(1,:) = [];
   T_before = T_before + t_mid;
   step_MW4 = traj_control(P_mid_1,P_mid_2,t_mid,Freq,theta5_mid,T_before);
   step_MW4(1,:) = [];
   T_before = T_before + t_mid;
   step_MW5 = traj_control(P_mid_2,P_mid_1,t_mid,Freq,theta5_mid,T_before);
   step_MW5(1,:) = [];
   T_before = T_before + t_mid;
Step_M = [step_trans_1;step_MW1;step_MW2;step_MW3;step_MW4;step_MW5];
%%%%%%%====================================================================
   %%% transition work position  
   step_trans_2 = traj_control_Joint(P_mid_1,P_right,t_transition,Freq,[theta5_mid,theta5_right],T_before);
   step_trans_2(1,:) = [];
   T_before = T_before + t_transition;
 %%% Right Space
   step_RW1 = traj_control(P_right,P_right_1,t_right,Freq,theta5_right,T_before);
   step_RW1(1,:) = [];
   T_before = T_before + t_right;
   step_RW2 = traj_control(P_right_1,P_right_2,t_right,Freq,theta5_right,T_before);
   step_RW2(1,:) = [];
   T_before = T_before + t_right;
   step_RW3 = traj_control(P_right_2,P_right_1,t_right,Freq,theta5_right,T_before);
   step_RW3(1,:) = [];
   T_before = T_before + t_right;
   step_RW4 = traj_control(P_right_1,P_right_2,t_right,Freq,theta5_right,T_before);
   step_RW4(1,:) = [];
   T_before = T_before + t_right;
   step_RW5 = traj_control(P_right_2,P_right_1,t_right,Freq,theta5_right,T_before);
   step_RW5(1,:) = [];
   T_before = T_before + t_right;
Step_R = [step_trans_2;step_RW1;step_RW2;step_RW3;step_RW4;step_RW5];
%%%%%%%====================================================================
 %%% transition work position  
   step_trans_3 = traj_control_Joint(P_right_1,P_left,t_transition,Freq,[theta5_right,theta5_left],T_before);
   step_trans_3(1,:) = [];
   T_before = T_before + t_transition;
 %%% Left Space
   step_LW1 = traj_control(P_left,P_left_1,t_left,Freq,theta5_left,T_before);
   step_LW1(1,:) = [];
   T_before = T_before + t_left;
   step_LW2 = traj_control(P_left_1,P_left_2,t_left,Freq,theta5_left,T_before);
   step_LW2(1,:) = [];
   T_before = T_before + t_left;
   step_LW3 = traj_control(P_left_2,P_left_1,t_left,Freq,theta5_left,T_before);
   step_LW3(1,:) = [];
   T_before = T_before + t_left;
   step_LW4 = traj_control(P_left_1,P_left_2,t_left,Freq,theta5_left,T_before);
   step_LW4(1,:) = [];
   T_before = T_before + t_left;
   step_LW5 = traj_control(P_left_2,P_left_1,t_left,Freq,theta5_left,T_before);
   step_LW5(1,:) = [];
   T_before = T_before + t_left;
Step_L = [step_trans_3;step_LW1;step_LW2;step_LW3;step_LW4;step_LW5];
   
 %%% Ending Pos
   step_End = traj_control_Joint(P_left_1,P0,t_transition,Freq,[theta5_left,theta5_mid],T_before);
   step_End(1,:) = [];
   T_before = T_before + t_transition;
   step = [
           step_0;
           Step_M;
           Step_R;
           Step_L;
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

% %% Simulation
% figure(3);
% pause(10);
% theta6_Time = zeros(size(theta1_Time));
% for i = 1:1:length(theta1_Time) 
% pause(1)
%  q = [theta1_Time(i)/180*pi theta2_Time(i)/180*pi theta3_Time(i)/180*pi theta4_Time(i)/180*pi theta5_Time(i)/180*pi theta6_Time(i)/180*pi];
%  Arm_2.plot(q);
% end

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
fileID = fopen('matlabToArduino_2.txt','w');

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
