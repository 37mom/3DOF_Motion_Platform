function [theta, phi] = Robot_Inverse_Kinematics (alpha, beta,EE,length,theta_0)
    
x_ee = EE(1); y_ee = EE(2); z_ee= EE(3);
l1 = length(1); l2 = length(2); l = length(3); d = length(4)
epsi = 120;
theta_1_0 = theta_0(1);
theta_2_0 = theta_0(2);
theta_3_0 = theta_0(3);


x_p1 = 0;
x_p2 = x_ee + d*cosd(beta)/2;
x_p3 = x_ee - d*cosd(beta)/2;
y_p1 = y_ee - d*cosd(alpha)/sqrt(3);
y_p2 = -y_p1/2 + 3*y_ee/2;
y_p3 = y_p2;
z_p1 = z_ee + d*(sind(beta)+sind(alpha)/sqrt(3))/2;
z_p2 = z_ee + d*(-sind(beta)+sind(alpha)/sqrt(3))/2;
z_p3 = z_ee - d*sind(alpha)/sqrt(3);


f1 = @(theta_1)(-l/sqrt(3) - l1 * sind(theta_1) + l2 * cosd(asind((z_p1 - l1 * cosd(theta_1)) / l2)) - y_p1);
f2 = @(theta_2)(l/(2*sqrt(3)) - l1 * sind(theta_2)*cosd(epsi) + l2 * cosd(asind((z_p2 - l1 * cosd(theta_2))/l2)) * cosd(epsi) - y_p2);
f3 = @(theta_3)(l/(2*sqrt(3)) - l1 * sind(theta_3)*cosd(2*epsi) + l2 * cosd(asind((z_p3 - l1 * cosd(theta_3)) / l2)) * cosd(2*epsi) - y_p3);


theta1 = fsolve(f1, theta_1_0);
theta2 = fsolve(f2, theta_2_0);
theta3 = fsolve(f3, theta_3_0);

phi1 = asind((z_p1-l1*cosd(theta1))/l2);
phi2 = asind((z_p2-l1*cosd(theta2))/l2);
phi3 = asind((z_p3-l1*cosd(theta3))/l2);

theta = [theta1 theta2 theta3];
phi = [phi1 phi2 phi3];


end