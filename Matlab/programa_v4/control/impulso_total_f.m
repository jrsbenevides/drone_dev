%%%%%%%%%%%%%%%%%% Federal University of Sao Carlos - UFSCar %%%%%%%%%%%%%%
%%%%%% Author: Isabella Cristina Souza Faria.                %%%%%%%%%%%%%%
%%%%%% e-mail: isamoreno2009@gmail.com                       %%%%%%%%%%%%%%
%%%%%% Professor Advisor: Roberto Santos Inoue.              %%%%%%%%%%%%%%
%%%%%% E-mail: rsinoue@ufscar.br                             %%%%%%%%%%%%%%
%%%%%% Date: January 20,2015                                 %%%%%%%%%%%%%%
%%%%%% Reference: Luukonen, Teppo. Modelling and control of quadcopter.
%%%%%% Aalto Universisty School of Science
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%% Revision 1: Roberto Santos Inoue - January 29, 2015   %%%%%%%%%%%%%%

function[T,z_error,dz_error]=impulso_total_f(dz_d,dz,z_d,z, theta_r,theta_p,quad)

%% The total thrust T affects the acceleration in the direction of the z axis and holds the quadcopter in the air.
%% [k_zd]= parameter for the derivative element of the PID controller.
%% [k_zp]= parameter for the proportional element of the PID controller.
%% [z]= position in the z axis.
%% [z_d]= desired position in the z axis.
%% [dz]= linear velocity in the z axis.
%% [dz_d]= desired linear velocity in the z axis.
%% [m]= mass of the quadcopter.
%% [g]= gravity acceleration.
%% [theta_r]= roll angle.
%% [theta_p]= pitch angle.

k_zd=quad.control.k_zd;
k_zp=quad.control.k_zp;
m=quad.model.m;
g=quad.model.g;

z_error = z_d-z;
dz_error = dz_d-dz;

T= (g+ k_zd*(dz_error)+ k_zp*(z_error))*(m/(cos(theta_r)*cos(theta_p)));

