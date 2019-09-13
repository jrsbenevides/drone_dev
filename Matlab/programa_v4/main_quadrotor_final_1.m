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

clear all
close all
clc
path(path,'.\trajectory')
path(path,'.\model')
path(path,'.\interfaces')
path(path,'.\figures')
path(path,'.\control')
%% Graphical User Interface
gui = guidata(gui_quad)


%% GUI settings
% Buttons settings
set(gui.pushbutton_waypoint,'Callback','pushbutton_waypoint_f')
set(gui.edit_posxd,'Callback','position_display_f')
set(gui.edit_posyd,'Callback','position_display_f')
set(gui.pushbutton_plot_gui,'Callback','plot_gui_graphics_f')
set(gui.pushbutton_plot,'Callback','plot_graphics_f')
set(gui.pushbutton_save_plot,'Callback','save_graphics_f')
set(gui.pushbutton_start,'Callback','experiment_f')
set(gui.pushbutton_stop,'Callback','flag_exp = 0;')


% Axes settings
set(gcf,'CurrentAxes',gui.axes1)
plot3(0,0,0)
dist = 3;
set(gui.axes1,'Xlim',[-dist dist],'Ylim',[-dist dist],'Zlim',[0 dist])
hold on;
grid;

%% Object of the quadrotor in the Axes 1
[quad1_obj] = load_object_f;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
set(gcf,'CurrentAxes',gui.axes2)
dist = 3;
set(gui.axes2,'Xlim',[-dist dist],'Ylim',[-dist dist],'Zlim',[0 dist])
hold on;
grid;

%% Object of the quadrotor in the Axes 2
[quad2_obj] = load_object_f;


%% Initial Conditions
x=0;
y=0;
z=0;
t = 0;
xf=str2double(get(gui.edit_posxd,'string'));
yf=str2double(get(gui.edit_posyd,'string'));
zf=str2double(get(gui.edit_poszd,'string'));
quad.model.g=9.81;
quad.model.k=0.00000289;
quad.model.b=0.000000114;
quad.model.l=0.25864;
quad.model.Ixx=0.031671826;
quad.model.Iyy=0.061646669;
quad.model.Izz=0.032310702;
quad.model.m=1.2211;
quad.control.k_zd=2.5;
quad.control.k_rd=1.75;
quad.control.k_pd=1.75;
quad.control.k_yd=1.75;
quad.control.k_zp=1.5;
quad.control.k_rp=6;
quad.control.k_pp=6;
quad.control.k_yp=6;
quad.control.traj.k_xp=1.85;
quad.control.traj.k_xd=0.75;
quad.control.traj.k_xdd=1;
quad.control.traj.k_yp=8.55;
quad.control.traj.k_yd=0.75;
quad.control.traj.k_ydd=1;
quad.control.traj.k_zp=1.85;
quad.control.traj.k_zd=0.75;
quad.control.traj.k_zdd=1;
dz_d=0;

s=[x;y;z];
sf=[xf;yf;zf];
ds=[0;0;0];
d2s=[0;0;0];
dsf=[0;0;0];
d2sf=[0;0;0];
s_ant = [x; y; z];

dz=0;
dy=0;
dx=0;
ds_ant = [dx;dy;dz];
d2x=0;
d2y=0;
d2z=0;

dtheta_r_d=0;
dtheta_r=0;
theta_r_d=0;
theta_r=0;
dtheta_p_d=0;
dtheta_p=0;
theta_p_d=0;
theta_p=0;
dtheta_y_d=0;
dtheta_y=0;
theta_y_d=pi/180*str2double(get(gui.edit_yawd,'string'));
theta_y=0;
d2theta_r=0;
d2theta_p=0;
d2theta_y=0;
dn_ant=[dtheta_r;dtheta_p;dtheta_y];
n_ant=[theta_r;theta_p;theta_y];
quad.model.Ax=0.25;
quad.model.Ay=0.25;
quad.model.Az=0.25;

set(gui.edit_time,'string',t)

set(gui.edit_posx,'string',x)
set(gui.edit_posy,'string',y)
set(gui.edit_posz,'string',z)

set(gui.edit_roll,'string',theta_r*180/pi)
set(gui.edit_pitch,'string',theta_p*180/pi)
set(gui.edit_yaw,'string',theta_y*180/pi)



