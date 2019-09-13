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

function[tau_y, theta_y_error, dtheta_y_error]=torque_yaw_f(dtheta_y_d,dtheta_y,theta_y_d,theta_y,quad);
%% The tau_y contributes in the acceleration of angle theta_y, yaw angle.
%% [k_yd]= parameter for the derivative element of the PID controller.
%% [k_yp]= parameter for the proportional element of the PID controller.
%% [Izz]= inertia moment in the z axis.
%% [theta_y]= yaw angle.
%% [theta_y_d]= desired yaw angle.
%% [dtheta_y]= yaw velocity.
%% [dtheta_y_d]= desired yaw velocity.

k_yd=quad.control.k_yd;
k_yp=quad.control.k_yp;
Izz=quad.model.Izz;

theta_y_error = normalize_angle_f(theta_y_d-theta_y,-pi);
dtheta_y_error = dtheta_y_d-dtheta_y;
tau_y= (k_yd*(dtheta_y_error)+k_yp*(theta_y_error))*Izz;
