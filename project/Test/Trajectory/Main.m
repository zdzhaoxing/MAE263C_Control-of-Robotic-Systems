%%%%%% Main %%%%%%
clear 
clc
P0 = [0.12;0;0.14]; 
P1 = [0.12;0;0.075];
t1 = 1;
P2 = [0.16;0;0.075];
t2 = 0.5;
P3 = [0.20;0;0.075];
t3 = 0.5;
P4 = [0.16;0;0.075];
t4 = 0.5;
P5 = [0.20;0;0.075];
t5 = 0.5;
P6 = [0.16;0;0.075];
t6 = 0.5;
P7 = [0.20;0;0.075];
t7 = 0.5;
P8 = [0.16;0;0.075];
t8 = 0.5;
% P8 = [0.21;0.02;0.01];
% P9 = [0.23;0;0.01];
% P10 = [0.21;0.02;0.01];
% P11 = [0.23;0;0.01];
% P12 = [0.21;0.02;0.01];
% P13 = [0.23;0;0.01];
% P14 = [0.21;0.02;0.01];;

Freq = 10;
theta5 = 0;
theta5_f = pi;
theta5_o = 0;
theta5_change = [theta5_o,theta5_f];

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

step_1 = traj_control_Joint(P0,P1,1,10,theta5_change,t1);
step_2 = traj_control_Joint(P1,P2,0.5,5,theta5_change,t2);
step_3 = traj_control(P2,P3,0.5,5,theta5,t3);
step_4 = traj_control(P3,P4,0.5,5,theta5,t4);
step_5 = traj_control(P4,P5,0.5,5,theta5,t5);
step_6 = traj_control(P5,P6,0.5,5,theta5,t6);
step_7 = traj_control(P6,P7,0.5,5,theta5,t7);
step_8 = traj_control(P7,P8,0.5,5,theta5,t8);
% step_9 = traj_control(P8,P9,0.5,5,theta5,4.5);
% step_10 = traj_control(P9,P10,0.5,5,theta5,5);
% step_11 = traj_control(P10,P11,0.5,5,theta5,5.5);
% step_12 = traj_control(P11,P12,0.5,5,theta5,6);
% step_13 = traj_control(P12,P13,0.5,5,theta5,6.5);
% step_14 = traj_control(P13,P14,0.5,5,theta5,7);

% step_2 = traj_control_Joint(P1,P2,T1,Freq_1,theta5_change,1);
% step = [step_1;step_2];
step = [step_1;step_2;step_3;step_4;step_5;step_6;step_7;step_8];
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
theta6_Time = step(:,7);
% for i = 1:1:length(theta1_Time) 
% pause(1)
%  q = [theta1_Time(i)/180*pi theta2_Time(i)/180*pi theta3_Time(i)/180*pi theta4_Time(i)/180*pi theta5_Time(i)/180*pi theta6_Time(i)/180*pi];
%  Arm_2.plot(q);
% end
theta1_Time = theta1_Time + 47;
theta2_Time = 180 - theta2_Time - 58;
theta3_Time = 180 + theta3_Time + 22;
theta4_Time = 90 + theta4_Time - 34-11;
% theta_Time = [T,theta1_Time,theta2_Time,theta3_Time,theta4_Time,theta5_Time,theta6_Time];
theta_Time = [];
Theta_Time = [T.' ; theta1_Time.' ; theta2_Time.' ; theta3_Time.' ; theta4_Time.' ; theta5_Time.' ; theta6_Time.'];

%write data to arduino in a .txt file
%to use, open the .txt file and copy paste to arduino
fileID = fopen('matlabToArduino_1.txt','w');

% Theta_Time = zeros(6,2);
col = size(Theta_Time,2);
num = 7 * col;


for j = 1:7
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