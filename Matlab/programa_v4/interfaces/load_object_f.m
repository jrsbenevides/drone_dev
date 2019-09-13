%%%%%%%%%%%%%%%%%% Universidade Federal de S�o Carlos %%%%%%%%%%%%%%%%%%%%%
%%%%%% Autora: Isabella Cristina Souza Faria.%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%% e-mail: isamoreno2009@gmail.com %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%% Orientador: Roberto Santos Inoue.%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%% e-mail: rsinoue@ufscar.br %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%% data: 20/01/2015 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function [corpo] = load_object_f
% Esta fun��o cria um objeto
% Roberto Santos Inoue
% Data: 18/03/2012
l_x = 0.4;
l_y = 0.2;
l_z = 0.1;

corpo.vertices(1,:) = [l_x, 0, 0]; 
corpo.vertices(2,:) = [0, l_y, 0]; 
corpo.vertices(3,:) = [0,-l_y, 0]; 
corpo.vertices(4,:) = [0, 0, l_z]; 
corpo.vertices(5,:) = [0, 0,-l_z];
corpo.vertices(6,:) = [-l_x, 0, 0]; 
corpo.faces = [1 2 5; 1 3 5; 1 3 4; 1 2 4; 6 2 5; 6 3 5; 6 3 4];
corpo.n = size(corpo.vertices,1);

corpo.objeto = patch('Vertices',corpo.vertices,...
    'Faces',corpo.faces,'FaceColor','red');