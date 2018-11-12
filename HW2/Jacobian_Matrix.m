% Zhaoxing Deng
% 
% Function to computer Jacobian matrix
% 
function J = Jacobian_Matrix(th1,th2,th3,L1,L2,L3)

J(1,1) = - (L1*sin(th1)+L2*sin(th1+th2)+L3*sin(th1+th2+th3));
J(1,2) = - (L2*sin(th1+th2)+L3*sin(th1+th2+th3));
J(1,3) = - L3*sin(th1+th2+th3);

J(2,1) = L1*cos(th1)+L2*cos(th1+th2)+L3*cos(th1+th2+th3);
J(2,2) = L2*cos(th1+th2)+L3*cos(th1+th2+th3);
J(2,3) = L3*cos(th1+th2+th3);

J(3,:) = [1;1;1]
end