%%%%%% Joint Spaces(faster)
%%%%%% Origin = [xo,yo,zo]: Original position of the end-effector
%%%%%% Final = [xf,yf,zf]: Final position of the end-effector
%%%%%% Time = Time from the original to the final
%%%%%% frequency = control frequency
%%%%%% theta5_change = [theta5_o,theta5_f]
%%%%%% d1 = 向下伸长的距离
%%%%%% d2 = 向外的距离(半径)
function TAV = traj_control_Joint(Origin,Final,Time,frequency,theta5_change,T_before)
%%%%%%%%% Read positions %%%%%%%%%% 
x_o= Origin(1);
y_o= Origin(2);
z_o= Origin(3);
x_f= Final(1);
y_f= Final(2);
z_f= Final(3);
%%%%%%% Set constants %%%%%%%%%%%%%
L1 = 9.3/100;
L2 = 2.1/100;
L3 = 10.45/100;
L4 = 9.7/100;
L5 = 2.2/100;
L6 = 7/100;

Num_dot = Time * frequency;
theta234 = 0;
%%%%%%%%% Map Operational Space to Joint Space %%%%%%%%
    %theta1%%%%%%%% =========================
       theta1_o = atan2(y_o,x_o);  %%%%%theta is not equal to 
    %theta3%%%%%%%% =========================
       if mod(theta1_o,pi)== 0
           K1 = x_o/cos(theta1_o) - L2 - L5*cos(theta234) - L6*sin(theta234);
       else 
           K1 = y_o/sin(theta1_o) - L2 - L5*cos(theta234) - L6*sin(theta234);
       end
       K2 = z_o - L1 - L5*sin(theta234) + L6*cos(theta234);
       cos3 = (K1^2+K2^2-L3^2-L4^2)/(2*L3*L4);
       theta3_o = -acos(cos3);
    %theta2%%%%%%%% ========================= MAE263A Slide#8 Case#6
       a_temp = L3+L4*cos(theta3_o);
       c_temp = -L4*sin(theta3_o);
       d_temp = K1;
       e_temp = L4*sin(theta3_o);
       f_temp = L3+L4*cos(theta3_o);
       g_temp = K2;
       sin2 = a_temp*g_temp-d_temp*e_temp;
       cos2 = d_temp*f_temp-c_temp*g_temp;
       theta2_o = atan2(sin2,cos2);
    %theta4%%%%%%%% =========================
       theta4_o = theta234 - theta2_o - theta3_o;
       if theta4_o>pi
           theta4_o = theta4_o - 2*pi;
       end
    %theta5%%%%%%%% =========================
       theta5_o = theta5_change(1);
%%%%%% Final %%%%%%%%%%%%%%%
    %theta1%%%%%%%% =========================
       theta1_f = atan2(y_f,x_f);  %%%%%theta is not equal to 
    %theta3%%%%%%%% =========================
       if mod(theta1_f,pi)== 0
           K1 = x_f/cos(theta1_f) - L2 - L5*cos(theta234) - L6*sin(theta234);
       else 
           K1 = y_f/sin(theta1_f) - L2 - L5*cos(theta234) - L6*sin(theta234);
       end
       K2 = z_f - L1 - L5*sin(theta234) + L6*cos(theta234);
       cos3 = (K1^2+K2^2-L3^2-L4^2)/(2*L3*L4);
       theta3_f = -acos(cos3);
    %theta2%%%%%%%% ========================= MAE263A Slide#8 Case#6
       a_temp = L3+L4*cos(theta3_f);
       c_temp = -L4*sin(theta3_f);
       d_temp = K1;
       e_temp = L4*sin(theta3_f);
       f_temp = L3+L4*cos(theta3_f);
       g_temp = K2;
       sin2 = a_temp*g_temp-d_temp*e_temp;
       cos2 = d_temp*f_temp-c_temp*g_temp;
       theta2_f = atan2(sin2,cos2);
    %theta4%%%%%%%% =========================
       theta4_f = theta234 - theta2_f - theta3_f;
       if theta4_f>pi
           theta4_f = theta4_f - 2*pi;
       end
    %theta5%%%%%%%% =========================
       theta5_f = theta5_change(2);   
       
% %%%%%%%%% Set Variants %%%%%%%%%%%
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
% x = [];
% y = [];
% z = [];
% %%%%%%% Set constants %%%%%%%%%%%%%
% L1 = 9.3/100;
% L2 = 2.1/100;
% L3 = 10.45/100;
% L4 = 9.7/100;
% L5 = 2.2/100;
% L6 = 7/100;
% 
% Num_dot = Time * frequency;
% theta234 = 0;
%%%%% Thetas' Movement %%%%%
%%%% x_t = a/3*t^3 - a/2*t^2 + a_c
a1_c = theta1_o;
a1 = (theta1_f - a1_c)/(Time^3/3 - Time^2/2);
a2_c = theta2_o;
a2 = (theta2_f - a2_c)/(Time^3/3 - Time^2/2);
a3_c = theta3_o;
a3 = (theta3_f - a3_c)/(Time^3/3 - Time^2/2);
a4_c = theta4_o;
a4 = (theta4_f - a4_c)/(Time^3/3 - Time^2/2);
a5_c = theta5_o;
a5 = (theta5_f - a5_c)/(Time^3/3 - Time^2/2);
   for i = 0:1:Num_dot
       t = i/frequency;
       T = [T;t];
       %%%%%%% positions(operational space) %%%%%%
       theta1 = a1/3*t^3 - a1/2*t^2 + a1_c;
       theta1_Time = [theta1_Time;theta1];
       theta2 = a2/3*t^3 - a2/2*t^2 + a2_c;
       theta2_Time = [theta2_Time;theta2];
       theta3 = a3/3*t^3 - a3/2*t^2 + a3_c;
       theta3_Time = [theta3_Time;theta3];
       theta4 = a4/3*t^3 - a4/2*t^2 + a4_c;
       theta4_Time = [theta4_Time;theta4];
       theta5 = a5/3*t^3 - a5/2*t^2 + a5_c;
       theta5_Time = [theta5_Time;theta5]; 
       theta6 = 0;
       theta6_Time = [theta6_Time;theta6];
       %%%%%%% velocities(operational space)%%%%%%
       theta1_dot = a1*(t^2 - t);
       theta1_dot_Time = [theta1_dot_Time;theta1_dot];
       theta2_dot = a2*(t^2 - t);
       theta2_dot_Time = [theta2_dot_Time;theta2_dot];
       theta3_dot = a3*(t^2 - t);
       theta3_dot_Time = [theta3_dot_Time;theta3_dot];
       theta4_dot = a4*(t^2 - t);
       theta4_dot_Time = [theta4_dot_Time;theta4_dot];
       theta5_dot = a5*(t^2 - t);
       theta5_dot_Time = [theta5_dot_Time;theta5_dot];       
       theta6_dot = 0;
       theta6_dot_Time = [theta6_dot_Time;theta6_dot];       
   end
   
A = [theta1_Time,theta2_Time,theta3_Time,theta4_Time,theta5_Time,theta6_Time];
V = [theta1_dot_Time,theta2_dot_Time,theta3_dot_Time,theta4_dot_Time,theta5_dot_Time,theta6_dot_Time];
T = T+T_before;
TAV = [T,A,V];
end