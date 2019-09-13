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


function[d2n]=angular_acceleration_inertial_frame_f(J,tau_B, C,dtheta_r,dtheta_p,dtheta_y)

%% The angular acceleration of the inertial frame is equal the torque, less the C matrix multiplied for the angular velocity of the inertial frame, and all  this multiplied by the inverse of the jacobian matrix.
%% [d2n]= angular acceleration of the inertial frame.
%% [J]= jacobian matrix.
%% [tau_B]= consists of the torques tau_r,tau_p,tau_y in the direction of the corresponding body frame angles.
%% [C]= C matrix.
%% [dn]= angular velocity of the inertial frame.

dn=[dtheta_r;dtheta_p;dtheta_y];
d2n= inv(J)*(tau_B-(C*dn));