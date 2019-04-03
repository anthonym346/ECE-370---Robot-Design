L0 = 0.3;
L1 = 0.2;
L2 = 0.1;
theta0 = 0.4;
theta1 = 0.6;
theta2 = 1.2;

ex1 = L0 * cos(theta0);
ex = ex1 + L1 * cos(theta0 + theta1);
ex2 = ex + L2 * cos(theta0 + theta1 + theta2)

ey1 = L0 * sin(theta0);
ey = ey1 + L1 * sin(theta0 + theta1);
ey2 = ey + L2 * sin(theta0 + theta1 + theta2)

T01 = [cos(theta0) -sin(theta0) 0 0; sin(theta0), cos(theta0) 0 0; 0 0 1 0; 0 0 0 1] * [1 0 0 L0;0 1 0 0; 0 0 1 0; 0 0 0 1];

T2 = [cos(theta1) -sin(theta1) 0 0; sin(theta1) cos(theta1) 0 0; 0 0 1 0; 0 0 0 1] * [1 0 0 L1; 0 1 0 0; 0 0 1 0; 0 0 0 1];


T3 = [cos(theta2) -sin(theta2) 0 0; sin(theta2) cos(theta2) 0 0; 0 0 1 0; 0 0 0 1] * [1 0 0 L2; 0 1 0 0; 0 0 1 0; 0 0 0 1];
T03 = T01*T2*T3

thetaX = rad2deg(atan2(T03(1,3),-T03(2,3)))

thetaY = rad2deg(atan2(T03(3,2),T03(3,3)))

thetaZ = rad2deg(atan2(T03(2,1),T03(1,1)))

Rx = [1,0,0;0,cosd(thetaX),-sind(thetaX);0,sind(thetaX),cosd(thetaX)]

Ry = [cosd(thetaY), 0, sind(thetaY); 0, 1, 0; -sind(thetaY), 0, cosd(thetaY)]
    
Rz = [cosd(thetaZ), -sind(thetaZ), 0; sind(thetaZ), cosd(thetaZ), 0; 0, 0, 1]

Rotation = Rx*Ry*Rz
