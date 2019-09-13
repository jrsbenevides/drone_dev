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


function[d2s]=linear_acceleration_inertial_frame_f(dx,dy,dz,T,theta_r,theta_p,theta_y,quad)

%% In the inertial frame, the centrifugal force is nollified. Thus, only the gravitational force and the magnitude and direction of the thrust are contributing in the aceleration of the quadcopter.
%% The linear acceleration of the inertial frame is equal the total thrust divided by the mass and multiplied by the rotation matrix and less the aerodynamical effects matrix multiplied by the linear velocity matrix.
%% [d2s]= linear acceleration of the inertial frame.
%% [g]= gravitational acceleration.
%% [T]= total thrust.
%% [m]= mass of the body.
%% [theta_p]= pitch angle.
%% [thera_r]= roll angle.
%% [theta_y]= yaw angle.
%% [Ax], [Ay] and [Az] = drag force coefficients for velocities in the corresponding directions of the inertial frame.
%% [dx;dy;dz] = linear velocity.
   
g=quad.model.g;
m=quad.model.m;
Ax=quad.model.Ax;
Ay=quad.model.Ay;
Az=quad.model.Az;

d2s= -g*[0;0;1] + (T/m)*[(cos(theta_y)*sin(theta_p)*cos(theta_r)+ sin(theta_y)*sin(theta_r));(sin(theta_y)*sin(theta_p)*cos(theta_r)-cos(theta_y)*sin(theta_r)); (cos(theta_p)*cos(theta_r))]-(1/m)*[Ax 0 0; 0 Ay 0; 0 0 Az]*[dx;dy;dz];
