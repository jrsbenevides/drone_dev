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

function[tau_p, theta_p_error, dtheta_p_error]=torque_pitch_f(dtheta_p_d, dtheta_p, theta_p_d, theta_p,quad)

%% The torque tau_p affects the acceleration of the angle theta_p, the pitch angle.
%% [k_pd]= parameter for the derivative element of the PID controller.
%% [k_pp]= parameter for the proportional element of the PID controller.
%% [Iyy]= inertia moment in the y axis.
%% [theta_p]= pitch angle.
%% [theta_p_d]= desired pitch angle.
%% [dtheta_p]= pitch velocity.
%% [dtheta_p_d]= desired pitch velocity.

k_pd=quad.control.k_pd;
k_pp=quad.control.k_pp;
Iyy=quad.model.Iyy;

theta_p_error = normalize_angle_f(theta_p_d-theta_p,-pi);
dtheta_p_error = dtheta_p_d-dtheta_p;

tau_p= (k_pd*(dtheta_p_error)+k_pp*(theta_p_error))*Iyy;