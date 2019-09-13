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


function[theta_r]=roll_angle_f(theta_y,x,xd,dx,dxd,d2x,d2xd,y,yd,dy,dyd,d2y,d2yd,z,zd,dz,dzd,d2z,d2zd,quad)

%% [theta_r]=roll angle;
%% [theta_y]= yaw angle;
%% [d2x]= acceleration in the x axis;
%% [dx]= velocity in the x axis;
%% [d2y]= acceleration in the y axis;
%% [dy]= velocity in the y axis;
%% [d2z]= acceleration in the z axis;
%% [dz]= velocity in the z axis;
%% [g]= gravity acceleration;
%% [m]= mass of the quadcopter;
%% [Ax], [Ay] and [Az]= drag force coefficients for velocities in the corresponding directions of the inertial frame.

Ax=quad.model.Ax;
Ay=quad.model.Ay;
Az=quad.model.Az;
g=quad.model.g;
m=quad.model.m;
k_xp=quad.control.traj.k_xp;
k_xd=quad.control.traj.k_xd;
k_xdd=quad.control.traj.k_xdd;
k_yp=quad.control.traj.k_yp;
k_yd=quad.control.traj.k_yd;
k_ydd=quad.control.traj.k_ydd;
k_zp=quad.control.traj.k_zp;
k_zd=quad.control.traj.k_zd;
k_zdd=quad.control.traj.k_zdd;
% d1=(d2x+(Ax*dx)/m);
% d2=(d2y+(Ay*dy)/m);
% d3=(d2z+(Az*dz)/m);

d1= k_xp*(xd-x) + k_xd*(dxd-dx) + k_xdd*(d2xd-d2x);
d2= k_yp*(yd-y) + k_yd*(dyd-dy) + k_ydd*(d2yd-d2y);
d3= k_zp*(zd-z) + k_zd*(dzd-dz) + k_zdd*(d2zd-d2z);

theta_r= asin((d1*sin(theta_y)-d2*cos(theta_y))/(d1^2+d2^2+(d3+g)^2));

