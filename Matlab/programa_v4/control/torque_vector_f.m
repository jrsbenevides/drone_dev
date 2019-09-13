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

function[tau_B]= torque_vector_f(tau_r,tau_p,tau_y);

%% The tau_B consists of the torques tau_r, tau_p and tau_y in the direction of the corresponding body frame angles.
%% [tau_r]= is the torque in the x direction. 
%% [tau_p]= is the torque in the y direction.
%% [tau_y]= is the torque in the z direction.

tau_B= [tau_r; tau_p; tau_y];

