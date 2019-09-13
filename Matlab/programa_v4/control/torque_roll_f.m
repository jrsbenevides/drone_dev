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

function[tau_r,theta_r_error,dtheta_r_error]=torque_roll_f(dtheta_r_d,dtheta_r,theta_r_d,theta_r,quad)

%% The torque tau_r has an affect on the acceleration of angle theta_r, the roll angle.
%% [k_rd]= parameter for the derivative element of the PID controller.
%% [k_rp]= parameter for the proportional element of the PID controller.
%% [theta_r]= roll angle.
%% [theta_r_d]= desired roll angle.
%% [dtheta_r_d]= desired roll velocity.
%% [dtheta_r]= roll velocity.
%% [Ixx]= inertia moment in the x axis.

k_rd=quad.control.k_rd;
k_rp=quad.control.k_rp;
Ixx=quad.model.Ixx;

theta_r_error = normalize_angle_f(theta_r_d-theta_r,-pi);
dtheta_r_error = dtheta_r_d-dtheta_r;

tau_r= (k_rd*(dtheta_r_error)+k_rp*(theta_r_error))*Ixx;