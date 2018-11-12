
L1 = 9.3;
L2 = 2.1;
L3 = 10.45;
L4 = 9.7;
L5 = 2.2;
L6 = 7;

Robot_263C(1) = Link([0  L1  0  0],'modified');
Robot_263C(2) = Link([0  0  L2  pi/2],'modified');
Robot_263C(3) = Link([0  0  L3  0],'modified');
Robot_263C(4) = Link([0  0  L4  0],'modified');
Robot_263C(5) = Link([0  L6  L5  pi/2],'modified');
Robot_263C(6) = Link([0  0  0  0],'modified');
Arm_2 = SerialLink(Robot_263C,'name','Robot_263C');
% theta1_Time = theta1_Time - 47;
q2 = [21,  21,  22,  24,  27,  34,  42,  52,  61,  67,   69];
theta2_Time = (180 - q2 - 58)/180*pi;
% theta3_Time = theta3_Time - 202;
% theta4_Time = theta4_Time - 56;

for i = 1:1:11 
pause(1)
 q = [0 q2(i) 0 0 0 0];
 Arm_2.plot(q);
end