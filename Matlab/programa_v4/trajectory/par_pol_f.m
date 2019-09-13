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

function [a] = par_pol_f(to,tf,qo,dqo,d2qo,qf,dqf,d2qf)
%function [a] = par_pol(to,tf,qo,dqo,d2qo,d3qo,qf,dqf,d2qf,d3qf)
% Fifth-degree polynomial
% q  = ao + a1*(t-to) + a2*(t-to)^2  + a3*(t-to)^3 + a4*(t-to)^4 + a5*(t-to)^5;

% q_v = [qo; dqo; d2qo; d3qo; qf; dqf; d2qf; d3qf];
q_v = [qo; dqo; d2qo; qf; dqf; d2qf];
delta_t = (tf-to);

A = [ 1 0 0 0 0 0;
      0 1 0 0 0 0;
      0 0 2 0 0 0;
      1 delta_t delta_t^2 delta_t^3 delta_t^4 delta_t^5;
      0 1     2*delta_t     3*delta_t^2  4*delta_t^3   5*delta_t^4;
      0 0     2             6*delta_t   12*delta_t^2  20*delta_t^3];
  
a= inv(A)*q_v;




