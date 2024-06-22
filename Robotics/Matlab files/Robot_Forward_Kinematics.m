function [EE_pos,phi,points,ll,alpha,beta] = Robot_Forward_Kinematics(length, phi_0, theta)
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
%% Passive angles initial conditions
phi_1 = phi_0(1);
phi_2 = phi_0(2);
phi_3 = phi_0(3);
%% Solving forward kinematics problem    
    tolr = 1e-15;                %error tolerance            
    for i = 1:1000
        [F,J] = forward_kin(phi_0, theta, length);
        phi = phi_0 - J\F;
        if(all(abs(phi-phi_0) < tolr))
            break;
        end
        phi_0 = phi;
    end
%% 
x_p1 =0;
y_p1 = -l/sqrt(3) - l1 * sind(theta_1) + l2 * cosd(phi(1));
z_p1 = l1 * cosd(theta_1) + l2 * sind(phi(1));

x_p2 = l/2 + l1 * sind(epsi) *sind(theta_2) - l2 * cosd(phi(2)) * sind(epsi);
y_p2 = l/(2*sqrt(3)) - l1 * sind(theta_2)*cosd(epsi) + l2 * cosd(phi(2)) * cosd(epsi);
z_p2 = l1 * cosd(theta_2) + l2 * sind(phi(2));
 
x_p3 = -l/2 + l1 * sind(2*epsi) *sind(theta_3) - l2 * cosd(phi(3)) * sind(2*epsi); 
y_p3 = l/(2*sqrt(3)) - l1 * sind(theta_3)*cosd(2*epsi) + l2 * cosd(phi(3)) * cosd(2*epsi);
z_p3 = l1 * cosd(theta_3) + l2 * sind(phi(3));

x_EE = (x_p1+x_p2+x_p3)/3;
y_EE = (y_p1+y_p2+y_p3)/3;
z_EE = (z_p1+z_p2+z_p3)/3;%+27.5+35;

l1 = sqrt((x_p2-x_p1)^2+(y_p2-y_p1)^2+(z_p2-z_p1)^2) ;
l2 = sqrt((x_p3-x_p1)^2+(y_p3-y_p1)^2+(z_p3-z_p1)^2) ;
l3 = sqrt((x_p2-x_p3)^2+(y_p2-y_p3)^2+(z_p2-z_p3)^2) ;
ll= [l1 l2 l3];

alpha = asind((((z_p1+z_p2)/2)-z_p3)/(d*sqrt(3)/2));
beta  = asind((z_p1-z_p2)/d);

point1 = [x_p1 y_p1 z_p1];
point2 = [x_p2 y_p2 z_p2];
point3 = [x_p3 y_p3 z_p3];
points = [point1;point2;point3];
EE_pos = [x_EE y_EE z_EE];
    
end