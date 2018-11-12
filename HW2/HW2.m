% Zhaoxing Deng, 005024802
%
% HW2.m
%

close all;
clear all;
clc;
% link lengths and manipulator configurations
L1 = 2; 
L2 = 1; 
L3 = 0.75;
th1 = (pi/180)*[0; -22.5; -45]; 
th2 = (pi/180)*[-0.05; -22.5; -67.5]; 
th3 = (pi/180)*[0; -45; -67.5]; 
% calculate the Jacobian matrix
J1 = Jacobian_Matrix(th1(1),th2(1),th3(1),L1,L2,L3);
J2 = Jacobian_Matrix(th1(2),th2(2),th3(2),L1,L2,L3);
J3 = Jacobian_Matrix(th1(3),th2(3),th3(3),L1,L2,L3);
% Perform Singular Value Decomposition
SingVals_v1 = svd(J1);
SingVals_v2 = svd(J2);
SingVals_v3 = svd(J3);
SingVals = [SingVals_v1,SingVals_v2,SingVals_v3];
SingVals
% calculate the endpoints of links 
L1_x = L1.*cos(th1);
L1_y = L1.*sin(th1);
L2_x = L1_x + L2.*cos(th1+th2);
L2_y = L1_y + L2.*sin(th1+th2);
L3_x = L2_x + L3.*cos(th1+th2+th3);
L3_y = L2_y + L3.*sin(th1+th2+th3);
% Creating a unit radius sphere for velocity analysis (rad/s) 
N = 29; 
% For generating three (N+1)x(N+1) matrices of coordinates 
[th1dot, th2dot, th3dot] = sphere(N);
% velocity manipulability ellipsoid of configuration 1
xdot1 = zeros(N+1,N+1);
ydot1 = zeros(N+1,N+1);
thdot1 = zeros(N+1,N+1);
for i = 1:N+1
    for j = 1:N+1
        v = J1*[th1dot(i,j);th2dot(i,j);th3dot(i,j)];
        xdot1(i,j) = v(1);
        ydot1(i,j) = v(2);
        thdot1(i,j) = v(3);
    end
end
% velocity manipulability ellipsoid of configuration 2
xdot2 = zeros(N+1,N+1);
ydot2 = zeros(N+1,N+1);
thdot2 = zeros(N+1,N+1);
for i = 1:N+1
    for j = 1:N+1
        v = J2*[th1dot(i,j);th2dot(i,j);th3dot(i,j)];
        xdot2(i,j) = v(1);
        ydot2(i,j) = v(2);
        thdot2(i,j) = v(3);
    end
end
% velocity manipulability ellipsoid of configuration 3 
xdot3 = zeros(N+1,N+1);
ydot3 = zeros(N+1,N+1);
thdot3 = zeros(N+1,N+1);
for i = 1:N+1
    for j = 1:N+1
        v = J3*[th1dot(i,j);th2dot(i,j);th3dot(i,j)];
        xdot3(i,j) = v(1);
        ydot3(i,j) = v(2);
        thdot3(i,j) = v(3);
    end
end

xdot = cat(3,xdot1,xdot2,xdot3);
ydot = cat(3,ydot1,ydot2,ydot3);
phidot = cat(3,thdot1,thdot2,thdot3);
% plot velocity manipulability ellipsoid
figure(1)
plot(0,0,'k.', 'MarkerSize', 20);hold on
for i=1:3
    plot([0,L1_x(i)],[0,L1_y(i)],'k', 'LineWidth', 2);hold on
    plot(L1_x(i), L1_y(i), 'k.', 'MarkerSize',20);hold on
    plot([L1_x(i),L2_x(i)],[L1_y(i),L2_y(i)],'k', 'LineWidth', 2);hold on
    plot(L2_x(i), L2_y(i), 'k.', 'MarkerSize',20);hold on
    plot([L2_x(i),L3_x(i)],[L2_y(i),L3_y(i)],'k', 'LineWidth', 2);hold on
    plot(L3_x(i), L3_y(i), 'k.', 'MarkerSize', 20);hold on
end
plot(L3_x(1)+xdot(:,:,1),L3_y(1)+ydot(:,:,1),'r-','linewidth',1);hold on
plot(L3_x(2)+xdot(:,:,2),L3_y(2)+ydot(:,:,2),'g-','linewidth',1);hold on
plot(L3_x(3)+xdot(:,:,3),L3_y(2)+ydot(:,:,3),'b-','linewidth',1);hold on
axis ([-5 8 -5 5]);
title('velocity manipulability ellipsoid');
xlabel('x or x-dot');ylabel('y or y-dot');
grid on;

JF1 = inv(J1');
JF2 = inv(J2');
JF3 = inv(J3');
% singular value decomposition on Jacobian inverse transpose
SingVals_f1 = svd(JF1);
SingVals_f2 = svd(JF2);
SingVals_f3 = svd(JF3);
SingVals_f = [SingVals_f1,SingVals_f2,SingVals_f3];
SingVals_f
% create a unit radius sphere for force analysis (arbitrary units) 
N = 29; 
% generate three (N+1)x(N+1) matrices of coordinates 
[tau1, tau2, tau3] = sphere(N); 

% force manipulability ellipsoid of configuration 1
fxdot1 = zeros(N+1,N+1);
fydot1 = zeros(N+1,N+1);
Mzdot1 = zeros(N+1,N+1);
for i = 1:N+1
    for j = 1:N+1       
        f = JF1*[tau1(i,j);tau2(i,j);tau3(i,j)];
        fxdot1(i,j) = f(1);
        fydot1(i,j) = f(2);
        Mzdot1(i,j) = f(3);
    end
end
% force manipulability ellipsoid of configuration 2
fxdot2 = zeros(N+1,N+1);
fydot2 = zeros(N+1,N+1);
Mzdot2 = zeros(N+1,N+1);
for i = 1:N+1
    for j = 1:N+1       
        f = JF2*[tau1(i,j);tau2(i,j);tau3(i,j)];
        fxdot2(i,j) = f(1);
        fydot2(i,j) = f(2);
        Mzdot2(i,j) = f(3);
    end
end
% force manipulability ellipsoid of configuration 3 
fxdot3 = zeros(N+1,N+1);
fydot3 = zeros(N+1,N+1);
Mdot3 = zeros(N+1,N+1);
for i = 1:N+1
    for j = 1:N+1        
        f = JF3*[tau1(i,j);tau2(i,j);tau3(i,j)];
        fxdot3(i,j) = f(1);
        fydot3(i,j) = f(2);
        Mdot3(i,j) = f(3);
    end
end

fxdot = cat(3,fxdot1,fxdot2,fxdot3);
fydot = cat(3,fydot1,fydot2,fydot3);
Mzdot = cat(3,Mzdot1,Mzdot2,Mdot3);

figure(2)
plot(0,0,'k.', 'MarkerSize', 20);hold on
for i=1:3
    plot([0,L1_x(i)],[0,L1_y(i)],'k', 'LineWidth', 2);hold on;
    plot(L1_x(i), L1_y(i), 'k.', 'MarkerSize',20);hold on
    plot([L1_x(i),L2_x(i)],[L1_y(i),L2_y(i)],'k', 'LineWidth', 2);hold on;
    plot(L2_x(i), L2_y(i), 'k.', 'MarkerSize',20);hold on
    plot([L2_x(i),L3_x(i)],[L2_y(i),L3_y(i)],'k', 'LineWidth', 2);hold on;
    plot(L3_x(i), L3_y(i), 'k.', 'MarkerSize', 20);hold on
end
plot(L3_x(1)+fxdot(:,:,1),L3_y(1)+fydot(:,:,1),'r-','linewidth',1);hold on
plot(L3_x(2)+fxdot(:,:,2),L3_y(2)+fydot(:,:,2),'g-','linewidth',1);hold on
plot(L3_x(3)+fxdot(:,:,3),L3_y(2)+fydot(:,:,3),'b-','linewidth',1);hold on
axis ([-5 8 -5 5]);
title('force manipulability ellipsoid');
xlabel('x or f_x-dot');ylabel('y or f_y-dot');
grid on;

for i = 1:3 
    figure(i+2)
    hSurface1 = surf(xdot(:,:,i), ydot(:,:,i), phidot(:,:,i)); 
    set(hSurface1,'FaceColor',[1 0 0],'FaceAlpha',0.35);    
    hold on;
    hSurface2 = surf(fxdot(:,:,i), fydot(:,:,i), Mzdot(:,:,i)); 
    set(hSurface2,'FaceColor',[0 0 1],'FaceAlpha',0.35);
    tempcmd = sprintf('title(''configuration i=%d'');',i);
    eval(tempcmd);
    xlabel('x-dot or f_x-dot');
    ylabel('y-dot or f_y-dot');
    zlabel('\phi-dot or M_z-dot');
    view(-37, 30);
end


