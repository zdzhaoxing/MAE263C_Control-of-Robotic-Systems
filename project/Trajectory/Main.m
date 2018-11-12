%%%%%% Main %%%%%%

P0 = [0.12;0;0.12]; 
P1 = [0.12;0;0.01];
P2 = [0.24;0;0.01];

T1 = 1;
Freq_1 = 10;
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
step_1 = traj_control(P0,P1,T1,Freq_1,theta5,0);
% step_2 = traj_control_Joint(P1,P2,T1,Freq_1,theta5_change,1);
% step = [step_1;step_2];
step = [step_1];
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
theta1_Time = theta1_Time + 47;
theta2_Time = 180 - theta2_Time - 58;
theta3_Time = 180 + theta3_Time + 22;
theta4_Time = 90 + theta4_Time - 34;
% theta_Time = [T,theta1_Time,theta2_Time,theta3_Time,theta4_Time,theta5_Time,theta6_Time];
theta_Time = [];
for i =1:1:length(T)
    theta_Time = [theta_Time;T(i);theta1_Time(i);theta2_Time(i);theta3_Time(i);theta4_Time(i);theta5_Time(i);theta6_Time(i)];
end
