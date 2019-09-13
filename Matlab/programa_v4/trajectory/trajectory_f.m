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


function [xd,yd,zd,dxd,dyd,dzd,d2xd,d2yd,d2zd] = trajectory_f(quad,dt,to,tf,so,dso,d2so,sf,dsf,d2sf);

m= quad.model.m
xo= so(1);
yo= so(2);
zo= so(3);
xf= sf(1);
yf= sf(2);
zf= sf(3);

dxo= dso(1);
dyo= dso(2);
dzo= dso(3);
dxf= dsf(1);
dyf= dsf(2);
dzf= dsf(3);

d2xo= d2so(1);
d2yo= d2so(2);
d2zo= d2so(3);
d2xf= d2sf(1);
d2yf= d2sf(2);
d2zf= d2sf(3);


[ax] = par_pol_f(to,tf,xo,dxo,d2xo,xf,dxf,d2xf);
[ay] = par_pol_f(to,tf,yo,dyo,d2yo,yf,dyf,d2yf);
[az] = par_pol_f(to,tf,zo,dzo,d2zo,zf,dzf,d2zf);

N= tf/dt;

for i=1:N
    
    t=i*dt;
    
    [xd(i),dxd(i),d2xd(i),d3xd(i)] = traj_pol_f(ax,t,to);
    [yd(i),dyd(i),d2yd(i),d3yd(i)] = traj_pol_f(ay,t,to);
    [zd(i),dzd(i),d2zd(i),d3zd(i)] = traj_pol_f(az,t,to);
       
end