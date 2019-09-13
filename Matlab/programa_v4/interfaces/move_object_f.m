
%%%%%%%%%%%%%%%%%% Universidade Federal de São Carlos %%%%%%%%%%%%%%%%%%%%%
%%%%%% Autora: Isabella Cristina Souza Faria.%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%% e-mail: isamoreno2009@gmail.com %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%% Orientador: Roberto Santos Inoue.%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%% e-mail: rsinoue@ufscar.br %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%% data: 20/01/2015 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [corpo] = move_objeto_f(corpo,att,pos)
% Move objeto
% [corpo] = move_objeto_f(corpo,att,pos)

R = rot_z_f(att(3))*rot_y_f(att(2))*rot_x_f(att(1));

for cont = 1:corpo.n
    corpo.vertices(cont,:) = (R*corpo.vertices(cont,:)' + pos)';    
end

set(corpo.objeto,'vertices',corpo.vertices);
