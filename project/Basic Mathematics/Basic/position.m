clc
clear

L1 = 9.3;
L2 = 2.1;
L3 = 10.45;
L4 = 9.7;
L5 = 2.2;
L6 = 7;
syms theta1 theta2 theta3 theta4 theta5
Midterm_6R(1) = Link([theta1  L1  0  0],'modified');
Midterm_6R(2) = Link([theta2  0  L2  pi/2],'modified');
Midterm_6R(3) = Link([theta3  0  L3  0],'modified');
Midterm_6R(4) = Link([theta4  0  L4  0],'modified');
Midterm_6R(5) = Link([theta5  L6  L5  pi/2],'modified');
Midterm_6R(6) = Link([0  0  0  0],'modified');
Arm_2 = SerialLink(Midterm_6R,'name','Arm_2');
T_1 = modified_transform(Midterm_6R(1));
T_2 = modified_transform(Midterm_6R(2));
T_3 = modified_transform(Midterm_6R(3));
T_4 = modified_transform(Midterm_6R(4));
T_5 = modified_transform(Midterm_6R(5));
T_6 = modified_transform(Midterm_6R(6));
T_transform = T_1*T_2*T_3*T_4*T_5*T_6
