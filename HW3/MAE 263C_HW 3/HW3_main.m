%**************************************************************************
% VERONICA J. SANTOS
% 4/20/18
% HW3_main.m
%
% This script file was originally created by L. Villani, G. Oriolo, and
% B. Siciliano in Feb. 2009.  It has been modified for MAE 263C HW #3.
%**************************************************************************

% Variable initialization
clear all;
global a k_r1 k_r2 pi_m pi_l

% load manipulator dynamic parameters without load mass
  param;
  pi_l = pi_m;

% gravity acceleration
  g = 9.81;

% friction matrix
  K_r = [k_r1,0;0,k_r2];
  F_v = K_r*[0.01,0;0,0.01]*K_r;

%   
  Tc = 0.001;

% controller gains
  K_p = 3700 * eye(2);  
  K_d = 750 * eye(2);
  
% desired position
%   q_d = [pi/4;-pi/2];
  q_d = [-pi;-3*pi/4];
 
% initial position
  q_i = q_d-0.1;

% duration of simulation
  t_d = 2.5;

% sample time for plots
  Ts = Tc
  
% sim hw3
% 
% figure(1)
% plot(t,q(:,1),'LineWidth',1,'Color','r'); grid on
% line([0,2.5],[q_d(1),q_d(1)],'linestyle',':');grid on
% xlabel('Time(s)');
% ylabel('Joint angles(rad)');
% title('Angle of Desired Posture q_d = [pi/4 ; -pi/2]')
% legend('Joint 1');
% 
% figure(2)
% plot(t,q(:,2),'LineWidth',1,'Color','r'); grid on
% line([0,2.5],[q_d(2),q_d(2)],'linestyle',':');grid on
% xlabel('Time(s)');
% ylabel('Joint angles(rad)');
% title('Angle of Desired Posture q_d = [pi/4 ; -pi/2]')
% legend('Joint 2')

sim hw3

figure(1)
plot(t,q(:,1),'LineWidth',1,'Color','r'); grid on
line([0,2.5],[q_d(1),q_d(1)],'linestyle',':');grid on
xlabel('Time(s)');
ylabel('Joint angles(rad)');
title('Angle of Desired Posture q_d = [-pi;-3*pi/4]')
legend('Joint 1');

figure(2)
plot(t,q(:,2),'LineWidth',1,'Color','r'); grid on
line([0,2.5],[q_d(2),q_d(2)],'linestyle',':');grid on
xlabel('Time(s)');
ylabel('Joint angles(rad)');
title('Angle of Desired Posture q_d = [-pi;-3*pi/4]')
legend('Joint 2')



  