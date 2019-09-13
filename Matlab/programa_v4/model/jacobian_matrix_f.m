
%%%%%%%%%%%%%%%%%% Universidade Federal de São Carlos %%%%%%%%%%%%%%%%%%%%%
%%%%%% Autora: Isabella Cristina Souza Faria.%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%% e-mail: isamoreno2009@gmail.com %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%% Orientador: Roberto Santos Inoue.%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%% e-mail: rsinoue@ufscar.br %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%% data: 20/01/2015 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function[J]=jacobian_matrix_f(theta_r, theta_p, theta_y,quad)

%% [J]= jacobian matrix.
%% [theta_p]= pitch angle.
%% [theta_r]= roll angle.
%% [theta_y]= yaw angle.
%% [Ixx]= inertia moment in the x axis.
%% [Iyy]= inertia moment in the y axis.
%% [Izz]= inertia moment in the z axis.


 Ixx=quad.model.Ixx;
 Iyy=quad.model.Iyy;
 Izz=quad.model.Izz;

J=[ Ixx 0 -Ixx*sin(theta_p);
    0 Iyy*(cos(theta_r)^2)+Izz*(sin(theta_r)^2) (Iyy-Izz)*cos(theta_r)*sin(theta_r)*cos(theta_p);
    -Ixx*sin(theta_p) (Iyy-Izz)*cos(theta_r)*sin(theta_r)*cos(theta_p) Ixx*(sin(theta_p)^2)+Iyy*(sin(theta_r)^2)*(cos(theta_p)^2)+ Izz*(cos(theta_r)^2)*(cos(theta_p)^2)];
