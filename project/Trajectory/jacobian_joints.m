function jacobian = jacobian_joints(q)
L1 = 9.3;
L2 = 2.1;
L3 = 10.45;
L4 = 9.7;
L5 = 2.2;
L6 = 7;
Robot(1) = Link([0  L1  0  0],'modified');
Robot(2) = Link([0  0  L2  pi/2],'modified');
Robot(3) = Link([0  0  L3  0],'modified');
Robot(4) = Link([0  0  L4  0],'modified');
Robot(5) = Link([0  L6  L5  pi/2],'modified');
Robot(6) = Link([0  0  0  0],'modified');
robot = SerialLink(Robot,'name','Robot');
jacobian = jacob0(robot, q);
end