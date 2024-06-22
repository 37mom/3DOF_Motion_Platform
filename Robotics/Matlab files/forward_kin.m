function [f,j] = forward_kin(phi_0, thetta, length)

%Function Variables
phi_1 = phi_0(1);
phi_2 = phi_0(2);
phi_3 = phi_0(3);
l1 = length(1);l2=length(2); l=length(3);  epsi=120 ;d =length(4);  
theta_1=thetta(1);  theta_2=thetta(2); theta_3=thetta(3);

%Function

f(1,1)=(((3^(1/2)*l)/2 - l2*cosd(phi_1) + l1*sind(theta_1) + l2*cosd(epsi)*cosd(phi_2) - l1*cosd(epsi)*sind(theta_2))^2 + (l1*cosd(theta_1) - l1*cosd(theta_2) + l2*sind(phi_1) - l2*sind(phi_2))^2 + (l/2 - l2*cosd(phi_2)*sind(epsi) + l1*sind(epsi)*sind(theta_2))^2) - d^2;
f(2,1)=((l/2 + l2*sind(2*epsi)*cosd(phi_3) - l1*sind(2*epsi)*sind(theta_3))^2 + (l1*cosd(theta_1) - l1*cosd(theta_3) + l2*sind(phi_1) - l2*sind(phi_3))^2 + ((3^(1/2)*l)/2 - l2*cosd(phi_1) + l1*sind(theta_1) + l2*cosd(2*epsi)*cosd(phi_3) - l1*cosd(2*epsi)*sind(theta_3))^2)-d^2;
f(3,1)=((l2*cosd(epsi)*cosd(phi_2) - l1*cosd(epsi)*sind(theta_2) - l2*cosd(2*epsi)*cosd(phi_3) + l1*cosd(2*epsi)*sind(theta_3))^2 + (l1*cosd(theta_2) - l1*cosd(theta_3) + l2*sind(phi_2) - l2*sind(phi_3))^2 + (l - l2*cosd(phi_2)*sind(epsi) + l1*sind(epsi)*sind(theta_2) + l2*sind(2*epsi)*cosd(phi_3) - l1*sind(2*epsi)*sind(theta_3))^2)-d^2;

j11 =2*l2*sind(phi_1)*((3^(1/2)*l)/2 - l2*cosd(phi_1) + l1*sind(theta_1) + l2*cosd(epsi)*cosd(phi_2) - l1*cosd(epsi)*sind(theta_2)) + 2*l2*cosd(phi_1)*(l1*cosd(theta_1) - l1*cosd(theta_2) + l2*sind(phi_1) - l2*sind(phi_2));
j12 =2*l2*sind(epsi)*sind(phi_2)*(l/2 - l2*cosd(phi_2)*sind(epsi) + l1*sind(epsi)*sind(theta_2)) - 2*l2*cosd(epsi)*sind(phi_2)*((3^(1/2)*l)/2 - l2*cosd(phi_1) + l1*sind(theta_1) + l2*cosd(epsi)*cosd(phi_2) - l1*cosd(epsi)*sind(theta_2)) - 2*l2*cosd(phi_2)*(l1*cosd(theta_1) - l1*cosd(theta_2) + l2*sind(phi_1) - l2*sind(phi_2));
j13 =0;
j21 =2*l2*sind(phi_1)*((3^(1/2)*l)/2 - l2*cosd(phi_1) + l1*sind(theta_1) + l2*cosd(2*epsi)*cosd(phi_3) - l1*cosd(2*epsi)*sind(theta_3)) + 2*l2*cosd(phi_1)*(l1*cosd(theta_1) - l1*cosd(theta_3) + l2*sind(phi_1) - l2*sind(phi_3));
j22 =0;
j23 =- 2*l2*cosd(phi_3)*(l1*cosd(theta_1) - l1*cosd(theta_3) + l2*sind(phi_1) - l2*sind(phi_3)) - 2*l2*cosd(2*epsi)*sind(phi_3)*((3^(1/2)*l)/2 - l2*cosd(phi_1) + l1*sind(theta_1) + l2*cosd(2*epsi)*cosd(phi_3) - l1*cosd(2*epsi)*sind(theta_3)) - 2*l2*sind(2*epsi)*sind(phi_3)*(l/2 + l2*sind(2*epsi)*cosd(phi_3) - l1*sind(2*epsi)*sind(theta_3));
j31 =0;
j32 =2*l2*cosd(phi_2)*(l1*cosd(theta_2) - l1*cosd(theta_3) + l2*sind(phi_2) - l2*sind(phi_3)) + 2*l2*sind(epsi)*sind(phi_2)*(l - l2*cosd(phi_2)*sind(epsi) + l1*sind(epsi)*sind(theta_2) + l2*sind(2*epsi)*cosd(phi_3) - l1*sind(2*epsi)*sind(theta_3)) - 2*l2*cosd(epsi)*sind(phi_2)*(l2*cosd(epsi)*cosd(phi_2) - l1*cosd(epsi)*sind(theta_2) - l2*cosd(2*epsi)*cosd(phi_3) + l1*cosd(2*epsi)*sind(theta_3));
j33 =2*l2*cosd(2*epsi)*sind(phi_3)*(l2*cosd(epsi)*cosd(phi_2) - l1*cosd(epsi)*sind(theta_2) - l2*cosd(2*epsi)*cosd(phi_3) + l1*cosd(2*epsi)*sind(theta_3)) - 2*l2*sind(2*epsi)*sind(phi_3)*(l - l2*cosd(phi_2)*sind(epsi) + l1*sind(epsi)*sind(theta_2) + l2*sind(2*epsi)*cosd(phi_3) - l1*sind(2*epsi)*sind(theta_3)) - 2*l2*cosd(phi_3)*(l1*cosd(theta_2) - l1*cosd(theta_3) + l2*sind(phi_2) - l2*sind(phi_3));
 
j = [j11 j12 j13;
     j21 j22 j23;
     j31 j32 j33];
end