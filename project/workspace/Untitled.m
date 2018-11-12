   
step = traj_control_Joint([0.12;0;0.12],[0.20;0;0.075],2,100,[0,0],1);
figure(1)
plot(T,step(:,2));hold on;
plot(T,step(:,3));hold on;
plot(T,step(:,4));hold on;
plot(T,step(:,5));hold on;
plot(T,step(:,6));hold on;
legend('Joint 1','Joint 2','Joint 3','Joint 4','Joint 5');
xlabel('Time(ms)');
ylabel('Angle')
grid on;
title('Desired Joint Angles');

figure(2)
plot(T,step(:,8));hold on;
plot(T,step(:,9));hold on;
plot(T,step(:,10));hold on;
plot(T,step(:,11));hold on;
plot(T,step(:,12));hold on;
legend('Joint 1','Joint 2','Joint 3','Joint 4','Joint 5');
xlabel('Time(ms)');
ylabel('Joint Velocities')
grid on;
title('Desired Joint Velocities');

step_1 = traj_control([0.12;0;0.12],[0.20;0;0.075],2,100,0,1);
figure(3)
plot(T,step_1(:,2));hold on;
plot(T,step_1(:,3));hold on;
plot(T,step_1(:,4));hold on;
plot(T,step_1(:,5));hold on;
plot(T,step_1(:,6));hold on;
legend('Joint 1','Joint 2','Joint 3','Joint 4','Joint 5');
xlabel('Time(ms)');
ylabel('Angle')
grid on;
title('Desired Joint Angles');

figure(4)
plot(T,step_1(:,8));hold on;
plot(T,step_1(:,9));hold on;
plot(T,step_1(:,10));hold on;
plot(T,step_1(:,11));hold on;
plot(T,step_1(:,12));hold on;
legend('Joint 1','Joint 2','Joint 3','Joint 4','Joint 5');
xlabel('Time(ms)');
ylabel('Joint Velocities')
grid on;
title('Desired Joint Velocities');