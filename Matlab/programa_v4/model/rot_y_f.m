%%%%%%%%%%%%%%%%%% Universidade Federal de São Carlos %%%%%%%%%%%%%%%%%%%%%
%%%%%% Autora: Isabella Cristina Souza Faria.%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%% e-mail: isamoreno2009@gmail.com %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%% Orientador: Roberto Santos Inoue.%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%% e-mail: rsinoue@ufscar.br %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%% data: 20/01/2015 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function [R_y] = rot_y_f(ang)

R_y = [cos(ang)  0 sin(ang);
       0         1 0;
       -sin(ang) 0 cos(ang)];
   
