%%%%%%%%%%%%%%%%%% Universidade Federal de São Carlos %%%%%%%%%%%%%%%%%%%%%
%%%%%% Autora: Isabella Cristina Souza Faria.%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%% e-mail: isamoreno2009@gmail.com %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%% Orientador: Roberto Santos Inoue.%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%% e-mail: rsinoue@ufscar.br %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%% data: 20/01/2015 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function [R_x] = rot_x_f(ang)
% Matriz de rotação no eixo x
% ang: ângulo de rotação da matriz de rotação
% R_x: matriz de rotação
% Grupo:

R_x = [1 0 0;
       0 cos(ang) -sin(ang);
       0 sin(ang)  cos(ang)];
   
