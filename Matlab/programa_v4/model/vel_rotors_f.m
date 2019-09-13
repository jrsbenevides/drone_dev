%%%%%%%%%%%%%%%%%% Federal University of Sao Carlos - UFSCar %%%%%%%%%%%%%%
%%%%%% Author: Isabella Cristina Souza Faria.                %%%%%%%%%%%%%%
%%%%%% E-mail: isamoreno2009@gmail.com                        %%%%%%%%%%%%%%
%%%%%% Professor Advisor: Roberto Santos Inoue.              %%%%%%%%%%%%%%
%%%%%% E-mail: rsinoue@ufscar.br                             %%%%%%%%%%%%%%
%%%%%% Date: January 20,2015                                 %%%%%%%%%%%%%%
%%%%%% Revision 1: Roberto Santos Inoue - January 29, 2015   %%%%%%%%%%%%%%


function[omega_square] = vel_rotors_f(T,tau_r,tau_p,tau_y,quad);
%% [omega_square] or [omega1_square; omega2_square;omega3_square;omega4_square]= angular velocities of rotors.
%% [T]= total thrust.
%% [k]= lift constant.
%% [b]= drag constant.
%% [l]= distance between the rotor and the center of mass of the quadcopter.
%% [tau_r]= torque in the x direction.
%% [tau_p]= torque in the y direction.
%% [tau_y]= torque in the z direction.


k=quad.model.k;
b=quad.model.b;
l=quad.model.l;

omega1_square = T/(4*k) - tau_p/(2*k*l) - tau_y/(4*b);
omega2_square = T/(4*k) - tau_r/(2*k*l) + tau_y/(4*b);
omega3_square = T/(4*k) + tau_p/(2*k*l) - tau_y/(4*b);
omega4_square = T/(4*k) - tau_r/(2*k*l) + tau_y/(4*b);

omega_square = [omega1_square; omega2_square; omega3_square; omega4_square];