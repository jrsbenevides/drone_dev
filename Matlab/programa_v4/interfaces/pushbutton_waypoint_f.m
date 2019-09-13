
%%%%%%%%%%%%%%%%%% Universidade Federal de São Carlos %%%%%%%%%%%%%%%%%%%%%
%%%%%% Autora: Isabella Cristina Souza Faria.%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%% e-mail: isamoreno2009@gmail.com %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%% Orientador: Roberto Santos Inoue.%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%% e-mail: rsinoue@ufscar.br %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%% data: 20/01/2015 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


traj_opt = 1;

if get(gui.checkbox_ginput,'value')==1
    
    [xf,yf]=ginput(1);
    set(gui.edit_posxd,'string',xf)
    set(gui.edit_posyd,'string',yf)
    zf=str2double(get(gui.edit_poszd,'string'));
    theta_y_d=pi/180*str2double(get(gui.edit_yawd,'string'));
    te = str2double(get(gui.edit_time_exp,'string'))
    td = str2double(get(gui.edit_time_traj,'string'))
    dt = str2double(get(gui.edit_dt,'string'))
    N = te/dt;
    
elseif get(gui.checkbox_ginput,'value')==0
    
    xf=str2double(get(gui.edit_posxd,'string'));
    yf=str2double(get(gui.edit_posyd,'string'));
    zf=str2double(get(gui.edit_poszd,'string'));
    theta_y_d=pi/180*str2double(get(gui.edit_yawd,'string'));
    te = str2double(get(gui.edit_time_exp,'string'))
    td = str2double(get(gui.edit_time_traj,'string'))
    dt = str2double(get(gui.edit_dt,'string'))
    N = te/dt;
end