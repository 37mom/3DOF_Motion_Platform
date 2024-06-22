function [velocity, phi_dot] = Robot_Forward_Velocity_Kin(length, phi, theta, theta_dot)

%% Geomtry Inputs
l1 = length(1);         %Short link
l2 = length(2);         %Long link
l = length(3);          %Base length
d = length(4);          %Plate length
%% Input angles
theta_1 = theta(1);
theta_2 = theta(2);
theta_3 = theta(3);
epsi    = 120;
phi_1 = phi(1);
phi_2 = phi(2);
phi_3 = phi(3);
theta_1_dot = theta_dot(1);
theta_2_dot = theta_dot(2);
theta_3_dot = theta_dot(3);

%% Solving for phi_dots:
syms phi_1_dot phi_2_dot phi_3_dot  
% eq1 = ((3^(1/2)*l)/2 - l2*cosd(phi_1) + l1*sind(theta_1) + l2*cosd(epsi)*cosd(phi_2) - l1*cosd(epsi)*sind(theta_2))*2*(l2*sind(phi_1)*phi_1_dot+l1*cosd(theta_1)*theta_1_dot-l2*cosd(epsi)*sind(phi_2)*phi_2_dot-l1*cosd(epsi)*sind(theta_2)*theta_2_dot) + (l1*cosd(theta_1) - l1*cosd(theta_2) + l2*sind(phi_1) - l2*sind(phi_2))*2*(-l1*sind(theta_1)*theta_1_dot+l1*sind(theta_2)*theta_2_dot+l2*cosd(phi_1)*phi_1_dot-l2*cosd(phi_2)*phi_2_dot) + (l/2 - l2*cosd(phi_2)*sind(epsi) + l1*sind(epsi)*sind(theta_2))*2*(l2*sind(epsi)*sind(phi_2)*phi_2_dot+l1*sind(epsi)*cosd(theta_2)*theta_2_dot) ;
% eq2= (l/2 + l2*sind(2*epsi)*cosd(phi_3) - l1*sind(2*epsi)*sind(theta_3))*2*(-l2*sind(2*epsi)*sind(phi_3)*phi_3_dot-l1*sind(2*epsi)*cosd(theta_3)*theta_3_dot) + (l1*cosd(theta_1) - l1*cosd(theta_3) + l2*sind(phi_1) - l2*sind(phi_3))*2*(-l1*sind(theta_1)*theta_1_dot+l1*sind(theta_3)*theta_3_dot+l2*cosd(phi_1)*phi_1_dot-l2*cosd(phi_3)*phi_3_dot)+ ((3^(1/2)*l)/2 - l2*cosd(phi_1) + l1*sind(theta_1) + l2*cosd(2*epsi)*cosd(phi_3) - l1*cosd(2*epsi)*sind(theta_3))*2*(l2*sind(phi_1)*phi_1_dot+l1*cosd(theta_1)*theta_1_dot-l2*cosd(2*epsi)*sind(phi_3)*phi_3_dot-l1*cosd(2*epsi)*cosd(theta_3)*theta_3_dot);
% eq3= (l2*cosd(epsi)*cosd(phi_2) - l1*cosd(epsi)*sind(theta_2) - l2*cosd(2*epsi)*cosd(phi_3) + l1*cosd(2*epsi)*sind(theta_3))*2*(-l2*cosd(epsi)*sind(phi_2)*phi_2_dot - l1*cosd(epsi)*cosd(theta_2)*theta_2_dot + l2*cosd(2*epsi)*sind(phi_3)*phi_3_dot + l1*cosd(2*epsi)*cosd(theta_3)*theta_3_dot) + (l1*cosd(theta_2) - l1*cosd(theta_3) + l2*sind(phi_2) - l2*sind(phi_3))*2*(-l1*sind(theta_2)*theta_2_dot + l1*sind(theta_3)*theta_3_dot + l2*cosd(phi_2)*phi_2_dot - l2*cosd(phi_3)*phi_3_dot) + (l - l2*cosd(phi_2)*sind(epsi) + l1*sind(epsi)*sind(theta_2) + l2*sind(2*epsi)*cosd(phi_3) - l1*sind(2*epsi)*sind(theta_3))*2*(l2*sind(phi_2)*sind(epsi)*phi_2_dot + l1*sind(epsi)*cosd(theta_2)*theta_2_dot - l2*sind(2*epsi)*sind(phi_3)*phi_3_dot - l1*sind(2*epsi)*cosd(theta_3)*theta_3_dot);
eq1 =phi_1_dot*(2*l2*sind(phi_1)*((3^(1/2)*l)/2 - l2*cosd(phi_1) + l1*sind(theta_1) + l2*cosd(epsi)*cosd(phi_2) - l1*cosd(epsi)*sind(theta_2)) + 2*l2*cosd(phi_1)*(l1*cosd(theta_1) - l1*cosd(theta_2) + l2*sind(phi_1) - l2*sind(phi_2))) + theta_1_dot*(2*l1*cosd(theta_1)*((3^(1/2)*l)/2 - l2*cosd(phi_1) + l1*sind(theta_1) + l2*cosd(epsi)*cosd(phi_2) - l1*cosd(epsi)*sind(theta_2)) - 2*l1*sind(theta_1)*(l1*cosd(theta_1) - l1*cosd(theta_2) + l2*sind(phi_1) - l2*sind(phi_2))) - phi_2_dot*(2*l2*cosd(phi_2)*(l1*cosd(theta_1) - l1*cosd(theta_2) + l2*sind(phi_1) - l2*sind(phi_2)) + 2*l2*cosd(epsi)*sind(phi_2)*((3^(1/2)*l)/2 - l2*cosd(phi_1) + l1*sind(theta_1) + l2*cosd(epsi)*cosd(phi_2) - l1*cosd(epsi)*sind(theta_2)) - 2*l2*sind(epsi)*sind(phi_2)*(l/2 - l2*cosd(phi_2)*sind(epsi) + l1*sind(epsi)*sind(theta_2))) + theta_2_dot*(2*l1*sind(theta_2)*(l1*cosd(theta_1) - l1*cosd(theta_2) + l2*sind(phi_1) - l2*sind(phi_2)) - 2*l1*cosd(epsi)*cosd(theta_2)*((3^(1/2)*l)/2 - l2*cosd(phi_1) + l1*sind(theta_1) + l2*cosd(epsi)*cosd(phi_2) - l1*cosd(epsi)*sind(theta_2)) + 2*l1*cosd(theta_2)*sind(epsi)*(l/2 - l2*cosd(phi_2)*sind(epsi) + l1*sind(epsi)*sind(theta_2))); 
eq2 =phi_1_dot*(2*l2*sind(phi_1)*((3^(1/2)*l)/2 - l2*cosd(phi_1) + l1*sind(theta_1) + l2*cosd(2*epsi)*cosd(phi_3) - l1*cosd(2*epsi)*sind(theta_3)) + 2*l2*cosd(phi_1)*(l1*cosd(theta_1) - l1*cosd(theta_3) + l2*sind(phi_1) - l2*sind(phi_3))) - theta_3_dot*(2*l1*cosd(2*epsi)*cosd(theta_3)*((3^(1/2)*l)/2 - l2*cosd(phi_1) + l1*sind(theta_1) + l2*cosd(2*epsi)*cosd(phi_3) - l1*cosd(2*epsi)*sind(theta_3)) - 2*l1*sind(theta_3)*(l1*cosd(theta_1) - l1*cosd(theta_3) + l2*sind(phi_1) - l2*sind(phi_3)) + 2*l1*sind(2*epsi)*cosd(theta_3)*(l/2 + l2*sind(2*epsi)*cosd(phi_3) - l1*sind(2*epsi)*sind(theta_3))) + theta_1_dot*(2*l1*cosd(theta_1)*((3^(1/2)*l)/2 - l2*cosd(phi_1) + l1*sind(theta_1) + l2*cosd(2*epsi)*cosd(phi_3) - l1*cosd(2*epsi)*sind(theta_3)) - 2*l1*sind(theta_1)*(l1*cosd(theta_1) - l1*cosd(theta_3) + l2*sind(phi_1) - l2*sind(phi_3))) - phi_3_dot*(2*l2*cosd(phi_3)*(l1*cosd(theta_1) - l1*cosd(theta_3) + l2*sind(phi_1) - l2*sind(phi_3)) + 2*l2*cosd(2*epsi)*sind(phi_3)*((3^(1/2)*l)/2 - l2*cosd(phi_1) + l1*sind(theta_1) + l2*cosd(2*epsi)*cosd(phi_3) - l1*cosd(2*epsi)*sind(theta_3)) + 2*l2*sind(2*epsi)*sind(phi_3)*(l/2 + l2*sind(2*epsi)*cosd(phi_3) - l1*sind(2*epsi)*sind(theta_3))); 
eq3 =theta_3_dot*(2*l1*sind(theta_3)*(l1*cosd(theta_2) - l1*cosd(theta_3) + l2*sind(phi_2) - l2*sind(phi_3)) - 2*l1*sind(2*epsi)*cosd(theta_3)*(l - l2*cosd(phi_2)*sind(epsi) + l1*sind(epsi)*sind(theta_2) + l2*sind(2*epsi)*cosd(phi_3) - l1*sind(2*epsi)*sind(theta_3)) + 2*l1*cosd(2*epsi)*cosd(theta_3)*(l2*cosd(epsi)*cosd(phi_2) - l1*cosd(epsi)*sind(theta_2) - l2*cosd(2*epsi)*cosd(phi_3) + l1*cosd(2*epsi)*sind(theta_3))) - phi_3_dot*(2*l2*cosd(phi_3)*(l1*cosd(theta_2) - l1*cosd(theta_3) + l2*sind(phi_2) - l2*sind(phi_3)) + 2*l2*sind(2*epsi)*sind(phi_3)*(l - l2*cosd(phi_2)*sind(epsi) + l1*sind(epsi)*sind(theta_2) + l2*sind(2*epsi)*cosd(phi_3) - l1*sind(2*epsi)*sind(theta_3)) - 2*l2*cosd(2*epsi)*sind(phi_3)*(l2*cosd(epsi)*cosd(phi_2) - l1*cosd(epsi)*sind(theta_2) - l2*cosd(2*epsi)*cosd(phi_3) + l1*cosd(2*epsi)*sind(theta_3))) + phi_2_dot*(2*l2*cosd(phi_2)*(l1*cosd(theta_2) - l1*cosd(theta_3) + l2*sind(phi_2) - l2*sind(phi_3)) + 2*l2*sind(epsi)*sind(phi_2)*(l - l2*cosd(phi_2)*sind(epsi) + l1*sind(epsi)*sind(theta_2) + l2*sind(2*epsi)*cosd(phi_3) - l1*sind(2*epsi)*sind(theta_3)) - 2*l2*cosd(epsi)*sind(phi_2)*(l2*cosd(epsi)*cosd(phi_2) - l1*cosd(epsi)*sind(theta_2) - l2*cosd(2*epsi)*cosd(phi_3) + l1*cosd(2*epsi)*sind(theta_3))) - theta_2_dot*(2*l1*sind(theta_2)*(l1*cosd(theta_2) - l1*cosd(theta_3) + l2*sind(phi_2) - l2*sind(phi_3)) - 2*l1*cosd(theta_2)*sind(epsi)*(l - l2*cosd(phi_2)*sind(epsi) + l1*sind(epsi)*sind(theta_2) + l2*sind(2*epsi)*cosd(phi_3) - l1*sind(2*epsi)*sind(theta_3)) + 2*l1*cosd(epsi)*cosd(theta_2)*(l2*cosd(epsi)*cosd(phi_2) - l1*cosd(epsi)*sind(theta_2) - l2*cosd(2*epsi)*cosd(phi_3) + l1*cosd(2*epsi)*sind(theta_3)));


f = solve([eq1, eq2, eq3],[phi_1_dot, phi_2_dot, phi_3_dot]);

phi_dot(1) =double( f.phi_1_dot);
phi_dot(2) =double( f.phi_2_dot);
phi_dot(3) =double(f.phi_3_dot);

%% Calculating velocity of plate points and end effector
xp1_dot = 0;
xp2_dot = l1*sind(epsi)*cosd(theta_2)*theta_2_dot + l2*sind(epsi)*sind(phi_2)*phi_dot(2);
xp3_dot = l1*sind(2*epsi)*cosd(theta_3)*theta_3_dot + l2*sind(2*epsi)*sind(phi_3)*phi_dot(3);

yp1_dot = -l1*cosd(theta_1)*theta_1_dot - l2*sind(phi_1)*phi_dot(1);
yp2_dot = -l1*cosd(epsi)*cosd(theta_2)*theta_2_dot - l2*cosd(epsi)*sind(phi_2)*phi_dot(2);
yp3_dot = -l1*cosd(2*epsi)*cosd(theta_3)*theta_3_dot - l2*cosd(2*epsi)*sind(phi_3)*phi_dot(3);

zp1_dot = -l1*sind(theta_1)*theta_1_dot + l2*cosd(phi_1)*phi_dot(1);
zp2_dot = -l1*sind(theta_2)*theta_2_dot + l2*cosd(phi_2)*phi_dot(2);
zp3_dot = -l1*sind(theta_3)*theta_3_dot + l2*cosd(phi_3)*phi_dot(3);

xEE_dot = (xp1_dot+xp2_dot+xp3_dot)/3;
yEE_dot = (yp1_dot+yp2_dot+yp3_dot)/3;
zEE_dot = (zp1_dot+zp2_dot+zp3_dot)/3;

velocity = [xEE_dot yEE_dot zEE_dot];

end