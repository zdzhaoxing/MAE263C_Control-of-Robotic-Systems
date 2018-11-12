clear
clc
T = [];
theta1_Time = [];
theta2_Time = [];
theta3_Time = [];
theta4_Time = [];
theta5_Time = [];
theta6_Time = [];
theta1_dot_Time = [];
theta2_dot_Time = [];
theta3_dot_Time = [];
theta4_dot_Time = [];
theta5_dot_Time = [];
theta6_dot_Time = [];
x = [];
y = [];
z = [];
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

for i = 0:1:1000
    t = i/1000;
    T = [T;t];
    theta1 = 0;
    theta5 = 0;
    theta4 = -30/180*pi;
    theta6 = 0;
    x_t = 0.15;
    x = [x;x_t];
    y_t = 0;
    y = [y;y_t];
    z_t = 0.16*t^3-0.24*t^2+0.08;
    z = [z;z_t];
    x_dot_t = 0;
    y_dot_t = 0;
    z_dot_t = 0.48*t^2 - 0.48*t;
    sin2 = (z_t - L1 + L6 - L4*sin(0-theta4))/L3; 
    cos2 = (x_t/cos(theta1) - L2 - L5 - L4*cos(0-theta4))/L3;
    theta2 = atan2(sin2,cos2);
    theta3 = 0 - theta4 - theta2;
    cartesian = [x_dot_t;y_dot_t;z_dot_t;0;0;0];
    q_t = [theta1;theta2;theta3;theta4;theta5;theta6];
    theta_dot = jacobian_joints(q_t).'* cartesian;
    
    theta1_Time = [theta1_Time;theta1];
    theta5_Time = [theta5_Time;theta5];
    theta4_Time = [theta4_Time;theta4];
    theta6_Time = [theta6_Time;theta6];
    theta2_Time = [theta2_Time;theta2];
    theta3_Time = [theta3_Time;theta3];
    
    theta1_dot_Time = [theta1_dot_Time;theta_dot(1)];
    theta2_dot_Time = [theta2_dot_Time;theta_dot(2)];
    theta3_dot_Time = [theta3_dot_Time;theta_dot(3)];
    theta4_dot_Time = [theta4_dot_Time;theta_dot(4)];
    theta5_dot_Time = [theta5_dot_Time;theta_dot(5)];
    theta6_dot_Time = [theta6_dot_Time;theta_dot(6)];
end


for i = 1001:1:3000
    t = i/1000-1;
    T = [T;t+1];
    theta1 = 0;
    theta5 = 0;
    theta4 = -30/180*pi;
    theta6 = 0;
    x_t = -0.15/4*t^3+0.45/4*t^2+0.15;
    x = [x;x_t];
    y_t = 0;
    y = [y;y_t];
    z_t = 0;
    z = [z;z_t];
    x_dot_t = -0.45/4*t^2+0.45/2*t;
    y_dot_t = 0;
    z_dot_t = 0;
    sin2 = (z_t - L1 + L6 - L4*sin(0-theta4))/L3; 
    cos2 = (x_t/cos(theta1) - L2 - L5 - L4*cos(0-theta4))/L3;
    theta2 = atan2(sin2,cos2);
    theta3 = 0 - theta4 - theta2;
    cartesian = [x_dot_t;y_dot_t;z_dot_t;0;0;0];
    q_t = [theta1;theta2;theta3;theta4;theta5;theta6];
    theta_dot = jacobian_joints(q_t).'* cartesian;
    
    theta1_Time = [theta1_Time;theta1];
    theta5_Time = [theta5_Time;theta5];
    theta4_Time = [theta4_Time;theta4];
    theta6_Time = [theta6_Time;theta6];
    theta2_Time = [theta2_Time;theta2];
    theta3_Time = [theta3_Time;theta3];
    
    theta1_dot_Time = [theta1_dot_Time;theta_dot(1)];
    theta2_dot_Time = [theta2_dot_Time;theta_dot(2)];
    theta3_dot_Time = [theta3_dot_Time;theta_dot(3)];
    theta4_dot_Time = [theta4_dot_Time;theta_dot(4)];
    theta5_dot_Time = [theta5_dot_Time;theta_dot(5)];
    theta6_dot_Time = [theta6_dot_Time;theta_dot(6)];
end
figure(10)
Arm_2.plot(q_t.');
for i = 3001:1:3500
    t = i/1000-3;
    T = [T;t+3];
    theta1 = 0;
    theta5 = 0;
    theta4 = -30/180*pi;
    theta6 = 0;
    x_t = 0.32*t^3-0.24*t^2+0.3;
    x = [x;x_t];
    y_t = 0;
    y = [y;y_t];
    z_t = 0;
    z = [z;z_t];
    x_dot_t = 0.96*t^2-0.48*t;
    y_dot_t = 0;
    z_dot_t = 0;
    sin2 = (z_t - L1 + L6 - L4*sin(0-theta4))/L3; 
    cos2 = (x_t/cos(theta1) - L2 - L5 - L4*cos(0-theta4))/L3;
    theta2 = atan2(sin2,cos2);
    theta3 = 0 - theta4 - theta2;
    cartesian = [x_dot_t;y_dot_t;z_dot_t;0;0;0];
    q_t = [theta1;theta2;theta3;theta4;theta5;theta6];
    theta_dot = jacobian_joints(q_t).'* cartesian;
    
    theta1_Time = [theta1_Time;theta1];
    theta5_Time = [theta5_Time;theta5];
    theta4_Time = [theta4_Time;theta4];
    theta6_Time = [theta6_Time;theta6];
    theta2_Time = [theta2_Time;theta2];
    theta3_Time = [theta3_Time;theta3];
    
    theta1_dot_Time = [theta1_dot_Time;theta_dot(1)];
    theta2_dot_Time = [theta2_dot_Time;theta_dot(2)];
    theta3_dot_Time = [theta3_dot_Time;theta_dot(3)];
    theta4_dot_Time = [theta4_dot_Time;theta_dot(4)];
    theta5_dot_Time = [theta5_dot_Time;theta_dot(5)];
    theta6_dot_Time = [theta6_dot_Time;theta_dot(6)];
end

for i = 3501:1:4000
    t = i/1000-3.5;
    T = [T;t+3.5];
    theta1 = 0;
    theta5 = 0;
    theta4 = -30/180*pi;
    theta6 = 0;
    x_t = -0.32*t^3+0.24*t^2+0.28;
    x = [x;x_t];
    y_t = 0;
    y = [y;y_t];
    z_t = 0;
    z = [z;z_t];
    x_dot_t = -0.96*t^2+0.48*t;
    y_dot_t = 0;
    z_dot_t = 0;
    sin2 = (z_t - L1 + L6 - L4*sin(0-theta4))/L3; 
    cos2 = (x_t/cos(theta1) - L2 - L5 - L4*cos(0-theta4))/L3;
    theta2 = atan2(sin2,cos2);
    theta3 = 0 - theta4 - theta2;
    cartesian = [x_dot_t;y_dot_t;z_dot_t;0;0;0];
    q_t = [theta1;theta2;theta3;theta4;theta5;theta6];
    theta_dot = jacobian_joints(q_t).'* cartesian;
    
    theta1_Time = [theta1_Time;theta1];
    theta5_Time = [theta5_Time;theta5];
    theta4_Time = [theta4_Time;theta4];
    theta6_Time = [theta6_Time;theta6];
    theta2_Time = [theta2_Time;theta2];
    theta3_Time = [theta3_Time;theta3];
    
    theta1_dot_Time = [theta1_dot_Time;theta_dot(1)];
    theta2_dot_Time = [theta2_dot_Time;theta_dot(2)];
    theta3_dot_Time = [theta3_dot_Time;theta_dot(3)];
    theta4_dot_Time = [theta4_dot_Time;theta_dot(4)];
    theta5_dot_Time = [theta5_dot_Time;theta_dot(5)];
    theta6_dot_Time = [theta6_dot_Time;theta_dot(6)];
end

for i = 4001:1:4500
    t = i/1000-4;
    T = [T;t+4];
    theta1 = 0;
    theta5 = 0;
    theta4 = -30/180*pi;
    theta6 = 0;
    x_t = 0.32*t^3-0.24*t^2+0.3;
    x = [x;x_t];
    y_t = 0;
    y = [y;y_t];
    z_t = 0;
    z = [z;z_t];
    x_dot_t = 0.96*t^2-0.48*t;
    y_dot_t = 0;
    z_dot_t = 0;
    sin2 = (z_t - L1 + L6 - L4*sin(0-theta4))/L3; 
    cos2 = (x_t/cos(theta1) - L2 - L5 - L4*cos(0-theta4))/L3;
    theta2 = atan2(sin2,cos2);
    theta3 = 0 - theta4 - theta2;
    cartesian = [x_dot_t;y_dot_t;z_dot_t;0;0;0];
    q_t = [theta1;theta2;theta3;theta4;theta5;theta6];
    theta_dot = jacobian_joints(q_t).'* cartesian;
    
    theta1_Time = [theta1_Time;theta1];
    theta5_Time = [theta5_Time;theta5];
    theta4_Time = [theta4_Time;theta4];
    theta6_Time = [theta6_Time;theta6];
    theta2_Time = [theta2_Time;theta2];
    theta3_Time = [theta3_Time;theta3];
    
    theta1_dot_Time = [theta1_dot_Time;theta_dot(1)];
    theta2_dot_Time = [theta2_dot_Time;theta_dot(2)];
    theta3_dot_Time = [theta3_dot_Time;theta_dot(3)];
    theta4_dot_Time = [theta4_dot_Time;theta_dot(4)];
    theta5_dot_Time = [theta5_dot_Time;theta_dot(5)];
    theta6_dot_Time = [theta6_dot_Time;theta_dot(6)];
end

for i = 4501:1:5000
    t = i/1000-4.5;
    T = [T;t+4.5];
    theta1 = 0;
    theta5 = 0;
    theta4 = -30/180*pi;
    theta6 = 0;
    x_t = -0.32*t^3+0.24*t^2+0.28;
    x = [x;x_t];
    y_t = 0;
    y = [y;y_t];
    z_t = 0;
    z = [z;z_t];
    x_dot_t = -0.96*t^2+0.48*t;
    y_dot_t = 0;
    z_dot_t = 0;
    sin2 = (z_t - L1 + L6 - L4*sin(0-theta4))/L3; 
    cos2 = (x_t/cos(theta1) - L2 - L5 - L4*cos(0-theta4))/L3;
    theta2 = atan2(sin2,cos2);
    theta3 = 0 - theta4 - theta2;
    cartesian = [x_dot_t;y_dot_t;z_dot_t;0;0;0];
    q_t = [theta1;theta2;theta3;theta4;theta5;theta6];
    theta_dot = jacobian_joints(q_t).'* cartesian;
    
    theta1_Time = [theta1_Time;theta1];
    theta5_Time = [theta5_Time;theta5];
    theta4_Time = [theta4_Time;theta4];
    theta6_Time = [theta6_Time;theta6];
    theta2_Time = [theta2_Time;theta2];
    theta3_Time = [theta3_Time;theta3];
    
    theta1_dot_Time = [theta1_dot_Time;theta_dot(1)];
    theta2_dot_Time = [theta2_dot_Time;theta_dot(2)];
    theta3_dot_Time = [theta3_dot_Time;theta_dot(3)];
    theta4_dot_Time = [theta4_dot_Time;theta_dot(4)];
    theta5_dot_Time = [theta5_dot_Time;theta_dot(5)];
    theta6_dot_Time = [theta6_dot_Time;theta_dot(6)];
end

figure(1)
title('Joint Angles');
plot(T,theta1_Time);hold on
plot(T,theta2_Time);hold on
plot(T,theta3_Time);hold on
plot(T,theta4_Time);hold on
plot(T,theta5_Time);
grid on
legend('Joint1','Joint2','Joint3','Joint4','Joint5');
xlabel('Time(s)');
ylabel('Angles(rad)');

figure(2)
title('Joint Velocities');
plot(T,theta1_dot_Time);hold on
plot(T,theta2_dot_Time);hold on
plot(T,theta3_dot_Time);hold on
plot(T,theta4_dot_Time);hold on
plot(T,theta5_dot_Time);
grid on
legend('Joint1','Joint2','Joint3','Joint4','Joint5');
xlabel('Time(s)');
ylabel('Velocities(rad/s)');
