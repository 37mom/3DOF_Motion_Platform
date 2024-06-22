clc 
clear
close all

L = [100 280 255.9 263.11];  %L1= 100, L2= 280, L= 255.9, d= 263.11
phi = [80; 80; 80];        %passive angles initial conditions
theta_0 = [50; 50; 50];
% alpha = 5.9133;
% beta  = -3.1042;
% EE_pos = [0.0409   -0.5146  327.5020];

%% Solving Forward Kinematics
% for jj = 0:2:90 
    theta = [70; 10; 50];
    [EE_pos,phi,points,l,alpha,beta] = Robot_Forward_Kinematics(L, phi, theta);
%     plot_robo(theta(1), theta(2), theta(3), phi(1)-90, phi(2)-90, phi(3)-90);
% end

%% Solving Inverse Kinematics

% [theta, phi] = Robot_Inverse_Kinematics (alpha, beta,EE_pos,L,theta_0);
% plot_robo_1(theta,phi);



