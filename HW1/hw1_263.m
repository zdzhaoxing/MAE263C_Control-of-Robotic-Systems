%%% Zhaoxing Deng 005024802 %%%
clear all;

clc;
sim hw1
figure(1)
plot(tout,I_a,'linewidth',2);
grid on;
title('Current (A) vs. time(sec)');
xlabel('time/s');
ylabel('current/A');

figure(2)
plot(tout,omega,'linewidth',2);
grid on;
title('Angular velocity(rad/s) vs. time(sec)');
xlabel('time/s');
ylabel('Angular velocity rad/s');