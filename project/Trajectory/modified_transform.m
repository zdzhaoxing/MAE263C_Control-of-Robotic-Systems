function T = modified_transform(Link)
Theta = Link.theta;
A = Link.a;
D = Link.d;
Alpha = Link.alpha;
T = [cos(Theta) -sin(Theta) 0  A ; sin(Theta)*cos(Alpha) cos(Theta)*cos(Alpha) -sin(Alpha) -sin(Alpha)*D ; sin(Theta)*sin(Alpha) cos(Theta)*sin(Alpha) cos(Alpha) cos(Alpha)*D ; 0 0 0 1];
