%%%%%%%%MAE263C Project Basic Matrices%%%%%%%%%%%%%%%%%%%%%%%%

clear
syms L1 L2 L3 L5 L6 L4 theta1 theta2 theta3 theta4 theta5 theta6
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
T_transform = T_1*T_2*T_3*T_4*T_5*T_6;
R_transform = T_transform(1:3,1:3);
syms theta1_t1 theta1_t2 theta1_t3 theta1_t4 theta1_t5 theta1_t6
w_0 = [0 0 0].';
R01 = T_1(1:3,1:3);
w_1 = R01.'* w_0 + [0 0 theta1_t1].';
R12 = T_2(1:3,1:3);
w_2 = R12.'* w_1 + [0 0 theta1_t2].';
R23 = T_3(1:3,1:3);
w_3 = R23.'* w_2 + [0 0 theta1_t3].';
R34 = T_4(1:3,1:3);
w_4 = R34.'* w_3 + [0 0 theta1_t4].';
R45 = T_5(1:3,1:3);
w_5 = R45.'* w_4 + [0 0 theta1_t5].';
R56 = T_6(1:3,1:3);
w_6 = R56.'* w_5 + [0 0 theta1_t6].';

v_0 = [0 0 0].';
P01 = T_1(1:3,4);
v_1 = R01.'*(cross(w_0,P01) +v_0);
P12 = T_2(1:3,4);
v_2 = R12.'*(cross(w_1,P12) +v_1);
P23 = T_3(1:3,4);
v_3 = R23.'*(cross(w_2,P23) +v_2);
P34 = T_4(1:3,4);
v_4 = R34.'*(cross(w_3,P34) +v_3);
P45 = T_5(1:3,4);
v_5 = R45.'*(cross(w_4,P45) +v_4);
P56 = T_6(1:3,4);
v_6 = R56.'*(cross(w_5,P56) +v_5);

w_6 = simplify(w_6);
w_6 = collect(w_6,theta1_t6);
w_6 = collect(w_6,theta1_t5);
w_6 = collect(w_6,theta1_t4);
w_6 = collect(w_6,theta1_t3);
w_6 = collect(w_6,theta1_t2);
w_6 = collect(w_6,theta1_t1);

v_6 = simplify(v_6);
v_6 = collect(v_6,theta1_t6);
v_6 = collect(v_6,theta1_t5);
v_6 = collect(v_6,theta1_t4);
v_6 = collect(v_6,theta1_t3);
v_6 = collect(v_6,theta1_t2);
v_6 = collect(v_6,theta1_t1);

q = [theta1_t1,theta1_t2,theta1_t3,theta1_t4,theta1_t5,theta1_t6];
J_tool = jacobian([v_6;w_6],q);
R0_6 = [inv(R_transform),zeros(3,3);zeros(3,3),inv(R_transform)];

%%%%%%%%%%Jacobian Matrix
J_base = simplify(R0_6*J_tool);

%%%%%%%%%Lengths
L1 = 9.3;
L2 = 2.1;
L3 = 10.45;
L4 = 9.7;
L5 = 2.2;
L6 = 7;
%%%%%%%%%%%thetas
Robot(1) = Link([0  L1  0  0],'modified');
Robot(2) = Link([0  0  L2  pi/2],'modified');
Robot(3) = Link([0  0  L3  0],'modified');
Robot(4) = Link([0  0  L4  0],'modified');
Robot(5) = Link([0  L6  L5  pi/2],'modified');
Robot(6) = Link([0  0  0  0],'modified');
Arm_x = SerialLink(Robot,'name','Arm_2');
q = [-pi/3 pi/3 pi/12 pi/3 pi/4 0];
%%%%%%%%%%Analytical Jacobian
j0 = Arm_x.jacob0(q.','eul')


