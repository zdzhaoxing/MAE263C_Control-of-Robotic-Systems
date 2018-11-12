syms theta2
equ = 1 == sin(theta2+0.2)+sin(theta2);
% equ = 18 - 2.1 - 2.2 == 10.45*cos(theta2)+9.7*cos(theta2+1.1);
solve(equ, theta2)