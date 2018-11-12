%%%%%% Origin = [xo,yo,zo]: Original position of the end-effector
%%%%%% Final = [xf,yf,zf]: Final position of the end-effector
%%%%%% Time = Time from the original to the final
%%%%%% frequency = control frequency
%%%%%% d1 = 向下伸长的距离
%%%%%% d2 = 向外的距离(半径)
function TAV = traj_control(Origin,Final,Time,frequency,theta5,T_before)
%%%%%%%%% Read positions %%%%%%%%%% 
xo= Origin(1);
yo= Origin(2);
zo= Origin(3);
xf= Final(1);
yf= Final(2);
zf= Final(3);
theta6 = 0;
%%%%%%%%% Set Variants %%%%%%%%%%%
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
%%%%%%% Set constants %%%%%%%%%%%%%
L1 = 9.3/100;
L2 = 2.1/100;
L3 = 10.45/100;
L4 = 9.7/100;
L5 = 2.2/100;
L6 = 7/100;

Num_dot = Time * frequency;
theta234 = 0;
%%%%% Thetas' Movement %%%%%
%%%% x_t = a/3*t^3 - a/2*t^2 + a_c
a_c = xo;
a = (xf - a_c)/(Time^3/3 - Time^2/2);
b_c = yo;
b = (yf - b_c)/(Time^3/3 - Time^2/2);
c_c = zo;
c = (zf - c_c)/(Time^3/3 - Time^2/2);


%%%%% For trajectory contants %%%%%%


   for i = 0:1:Num_dot
       t = i/frequency;
       T = [T;t];
       %%%%%%% positions(operational space) %%%%%%
       x_t = a/3*t^3 - a/2*t^2 + a_c;
       x = [x;x_t];
       y_t = b/3*t^3 - b/2*t^2 + b_c;
       y = [y;y_t];
       z_t = c/3*t^3 - c/2*t^2 + c_c;
       z = [z;z_t];
       %%%%%%% velocities(operational space)%%%%%%
       x_dot_t = a*(t^2 - t);
       y_dot_t = b*(t^2 - t);
       z_dot_t = c*(t^2 - t);
       
       %%%%%% Angles(Joint Space)%%%%%%
    %theta1%%%%%%%% =========================
       theta1 = atan2(y_t,x_t);  %%%%%theta is not equal to 
    %theta3%%%%%%%% =========================
       if mod(theta1,pi)== 0
           K1 = x_t/cos(theta1) - L2 - L5*cos(theta234) - L6*sin(theta234);
       else 
           K1 = y_t/sin(theta1) - L2 - L5*cos(theta234) - L6*sin(theta234);
       end
       K2 = z_t - L1 - L5*sin(theta234) + L6*cos(theta234);
       cos3 = (K1^2+K2^2-L3^2-L4^2)/(2*L3*L4);
       theta3 = -acos(cos3);
    %theta2%%%%%%%% ========================= MAE263A Slide#8 Case#6
       a_temp = L3+L4*cos(theta3);
       c_temp = -L4*sin(theta3);
       d_temp = K1;
       e_temp = L4*sin(theta3);
       f_temp = L3+L4*cos(theta3);
       g_temp = K2;
       sin2 = a_temp*g_temp-d_temp*e_temp;
       cos2 = d_temp*f_temp-c_temp*g_temp;
       theta2 = atan2(sin2,cos2);
    %theta4%%%%%%%% =========================
       theta4 = theta234 - theta2 - theta3;
       if theta4>pi
           theta4 = theta4 - 2*pi;
       end
    %%%%%%%%% =========================
    %%%%%%% Map %%%%%%%%%%
        cartesian = [x_dot_t;y_dot_t;z_dot_t;0;0;0];
        q_t = [theta1;theta2;theta3;theta4;theta5;theta6];
        theta_dot = jacobian_joints(q_t).'* cartesian;
    %%%%%%%%%%%%%%%%%%%%%% Add the value     
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
A = [theta1_Time,theta2_Time,theta3_Time,theta4_Time,theta5_Time,theta6_Time];
V = [theta1_dot_Time,theta2_dot_Time,theta3_dot_Time,theta4_dot_Time,theta5_dot_Time,theta6_dot_Time];
T = T+T_before;
TAV = [T,A,V];
end