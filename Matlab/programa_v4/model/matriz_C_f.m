
%%%%%%%%%%%%%%%%%% Universidade Federal de São Carlos %%%%%%%%%%%%%%%%%%%%%
%%%%%% Autora: Isabella Cristina Souza Faria.%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%% e-mail: isamoreno2009@gmail.com %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%% Orientador: Roberto Santos Inoue.%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%% e-mail: rsinoue@ufscar.br %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%% data: 20/01/2015 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function[C]=matriz_C_f(theta_r,theta_p,theta_y, dtheta_r,dtheta_p,dtheta_y,quad)

%% The matrix C is the Coriolis term, containing the gyroscopic and centripetal terms.
%% [Iyy]= inertia moment in the y axis.
%% [Izz]= inertia moment in the z axis.
%% [Ixx]= inertia moment in the x axis.
%% [theta_r]= roll angle.
%% [theta_p]= pitch angle.
%% [theta_y]= yaw angle.

Ixx=quad.model.Ixx;
Iyy=quad.model.Iyy;
Izz=quad.model.Izz;

C_11= 0;
C_12= (Iyy-Izz)*(dtheta_p*cos(theta_r)*sin(theta_r) + dtheta_y*(sin(theta_r)^2)*cos(theta_p))+ (Izz-Iyy)*dtheta_r*(cos(theta_r)^2)*cos(theta_p) - Ixx*dtheta_y*cos(theta_p);
C_13= (Izz-Iyy)*dtheta_y*cos(theta_r)*sin(theta_r)*(cos(theta_p)^2);
C_21= (Izz-Iyy)*(dtheta_p*cos(theta_r)*sin(theta_r) + dtheta_y*sin(theta_r)*cos(theta_r)) + (Iyy-Izz)*dtheta_y*(cos(theta_r)^2)*cos(theta_p) + Ixx*dtheta_y*cos(theta_p);
C_22= (Izz-Iyy)*dtheta_r*cos(theta_r)*sin(theta_r);
C_23= -Ixx*dtheta_y*sin(theta_p)*cos(theta_p) + Iyy*dtheta_y*(sin(theta_r)^2)*sin(theta_p)*cos(theta_p) + Izz*dtheta_y*(cos(theta_r)^2)*sin(theta_p)*cos(theta_p);
C_31= (Iyy-Izz)*dtheta_y*(cos(theta_p)^2)*sin(theta_r)*cos(theta_r) - Ixx*dtheta_p*cos(theta_p);
C_32= (Izz-Iyy)*(dtheta_p*cos(theta_r)*sin(theta_r)*sin(theta_p) + dtheta_r*(sin(theta_r)^2)*cos(theta_p)) + (Iyy-Izz)*dtheta_r*(cos(theta_r)^2)*cos(theta_p) + Ixx*dtheta_y*sin(theta_p)*cos(theta_p) - Iyy*dtheta_y*(sin(theta_r)^2)*sin(theta_p)*cos(theta_p) - Izz*dtheta_y*(cos(theta_r)^2)*sin(theta_p)*cos(theta_p);
C_33= (Iyy-Izz)*dtheta_r*cos(theta_r)*sin(theta_r)*(cos(theta_p)^2) - Iyy*dtheta_p*(sin(theta_r)^2)*cos(theta_p)*sin(theta_p) -Izz*dtheta_p*(cos(theta_r)^2)*cos(theta_p)*sin(theta_p) + Ixx*dtheta_p*cos(theta_p)*sin(theta_p);


C= [ C_11 C_12 C_13; C_21 C_22 C_23; C_31 C_32 C_33];