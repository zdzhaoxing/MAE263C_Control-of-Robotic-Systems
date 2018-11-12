
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
 pause(1)
 q = [0 0 0 0 0 0];
 Arm_2.plot(q);
pause(1)
q = [0 0 0 pi/3 0 0];
 Arm_2.plot(q);

pause(1)
q = [0 0 0 pi/2 0 0];
 Arm_2.plot(q);

pause(1)