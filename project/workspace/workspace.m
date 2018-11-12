clear
clc

dot = [];
% for theta1 = -3*pi/4:0.1:3*pi/4
theta1 = 0; 
    for theta2 = -3*pi/4:0.1:3*pi/4
        for theta3 = -3*pi/4:0.1:3*pi/4
            for theta4 = -3*pi/4:0.1:3*pi/4
                x = (21*cos(theta1))/10 + (209*cos(theta1)*cos(theta2))/20 + 7*cos(theta4)*(cos(theta1)*cos(theta2)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2)) - (11*cos(theta4)*(cos(theta1)*sin(theta2)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)))/5 - (11*sin(theta4)*(cos(theta1)*cos(theta2)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2)))/5 - 7*sin(theta4)*(cos(theta1)*sin(theta2)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) - (97*cos(theta1)*sin(theta2)*sin(theta3))/10 + (97*cos(theta1)*cos(theta2)*cos(theta3))/10;
                y = (21*sin(theta1))/10 + (209*cos(theta2)*sin(theta1))/20 + 7*cos(theta4)*(cos(theta2)*sin(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2)) - (11*cos(theta4)*(sin(theta1)*sin(theta2)*sin(theta3) - cos(theta2)*cos(theta3)*sin(theta1)))/5 - (11*sin(theta4)*(cos(theta2)*sin(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2)))/5 - 7*sin(theta4)*(sin(theta1)*sin(theta2)*sin(theta3) - cos(theta2)*cos(theta3)*sin(theta1)) - (97*sin(theta1)*sin(theta2)*sin(theta3))/10 + (97*cos(theta2)*cos(theta3)*sin(theta1))/10;
                z = (209*sin(theta2))/20 + (97*cos(theta2)*sin(theta3))/10 + (97*cos(theta3)*sin(theta2))/10 + (11*cos(theta4)*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2)))/5 - 7*cos(theta4)*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3)) + 7*sin(theta4)*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2)) + (11*sin(theta4)*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3)))/5 + 93/10;
                dot = [dot;x,y,z];
            end
        end
    end
% end

figure(1)
plot(dot(:,1),dot(:,3));
xlabel('X');
ylabel('Z');
grid on

z0 = [];
for i = 1:1:110592
    if abs(dot(i,1)) < 0.001
        z0 = [z0;dot(i,3)];
    end
end

max_0 = max(z0)
min_0 = min(z0)