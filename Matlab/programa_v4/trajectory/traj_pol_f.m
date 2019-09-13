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


function [q,dq,d2q,d3q] = traj_pol_f(a,t,to)

ao = a(1);
a1 = a(2);
a2 = a(3);
a3 = a(4);
a4 = a(5);
a5 = a(6);

delta_t = t-to;

q  = ao + a1*delta_t + a2*delta_t^2  + a3*delta_t^3 + a4*delta_t^4 + a5*delta_t^5;
dq = a1 +  2*a2*delta_t + 3*a3*delta_t^2 + 4*a4*delta_t^3 + 5*a5*delta_t^4;
d2q = 2*a2 + 6*a3*delta_t + 12*a4*delta_t^2 + 20*a5*delta_t^3;
d3q =  6*a3 + 24*a4*delta_t + 60*a5*delta_t^2;

% q    =   ao +    a1*delta_t +    a2*delta_t^2 +     a3*delta_t^3 +     a4*delta_t^4 +    a5*delta_t^5 +   a6*delta_t^6 + a7*delta_t^7;
% dq   =   a1 +  2*a2*delta_t +  3*a3*delta_t^2 +   4*a4*delta_t^3 +   5*a5*delta_t^4 +  6*a6*delta_t^5 + 7*a7*delta_t^6;
% ddq  = 2*a2 +  6*a3*delta_t + 12*a4*delta_t^2 +  20*a5*delta_t^3 +  30*a6*delta_t^4 + 42*a7*delta_t^5;
% dddq = 6*a3 + 24*a4*delta_t + 60*a5*delta_t^2 + 120*a6*delta_t^3 + 210*a7*delta_t^4;