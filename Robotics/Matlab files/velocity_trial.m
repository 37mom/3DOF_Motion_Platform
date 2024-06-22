clc
clear all

theta =[40; 60; 70];
theta_dot = [2 2 2]*pi/180;
[EE_pos,phi,points,ll,alpha,beta] = Robot_Forward_Kinematics([100 280 255.9 263.11], [80; 80; 80],theta);

[velocity, phi_dot] = Robot_Forward_Velocity_Kin([100 280 255.9 263.11], phi, theta, theta_dot)