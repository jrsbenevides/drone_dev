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

to=0;
te = str2double(get(gui.edit_time_exp,'string'))
td = str2double(get(gui.edit_time_traj,'string'))
dt = str2double(get(gui.edit_dt,'string'))
N = te/dt;

traj_opt = 1;
flag_exp = 1
i = 0;
while(flag_exp)
    i = i+1
    if i == N
        flag_exp = 0;
    end
    
    t = dt*i;
    t_v(i) = t;

    
    if traj_opt == 1  
        sf=[xf;yf;zf];
        dsf=[0;0;0];
        d2sf=[0;0;0];
        to = t;
        to = t;
        tf = to+td;
        [ax,ay,az] = trajectory_par_pol_f(dt,to,tf,s,ds,d2s,sf,dsf,d2sf);
        traj_opt = 0;
    end
    
    if t<=tf
        [xd,dxd,d2xd,d3xd] = traj_pol_f(ax,t,to);
        [yd,dyd,d2yd,d3yd] = traj_pol_f(ay,t,to);
        [zd,dzd,d2zd,d3zd] = traj_pol_f(az,t,to);
    end
    
    xd_v(i)=xd;
    yd_v(i)=yd;
    zd_v(i)= zd;
    dxd_v(i)=dxd;
    dyd_v(i)=dyd;
    dzd_v(i)= dzd;
    d2xd_v(i)=d2xd;
    d2yd_v(i)=d2yd;
    d2zd_v(i)=d2zd;
    
    [theta_r_d]= roll_angle_f(theta_y,x,xd,dx,dxd,d2x,d2xd,y,yd,dy,dyd,d2y,d2yd,z,zd,dz,dzd,d2z,d2zd,quad);
    [theta_p_d]= pitch_angle_f(theta_y,x,xd,dx,dxd,d2x,d2xd,y,yd,dy,dyd,d2y,d2yd,z,zd,dz,dzd,d2z,d2zd,quad);
    
    theta_y_d_v(i)=theta_y_d*180/pi;
    theta_r_d_v(i)=theta_r_d*180/pi;
    theta_p_d_v(i)=theta_p_d*180/pi;
    
    %% Calc of the Impulse and of the Torques
    [T,z_error,dz_error]=impulso_total_f(dzd,dz,zd,z, theta_r,theta_p,quad);
    [tau_r,theta_r_error,dtheta_r_error]=torque_roll_f(dtheta_r_d,dtheta_r,theta_r_d,theta_r,quad);
    [tau_p, theta_p_error, dtheta_p_error]=torque_pitch_f(dtheta_p_d, dtheta_p, theta_p_d, theta_p,quad);
    [tau_y, theta_y_error, dtheta_y_error]=torque_yaw_f(dtheta_y_d,dtheta_y,theta_y_d,theta_y,quad);
    [tau_B]= torque_vector_f(tau_r,tau_p,tau_y);
    
    T_v(i) = T;
    z_error_v(i) = z_error;
    dz_error_v(i) = dz_error;
    
    tau_r_v(i) = tau_r;
    theta_r_error_v(i) = theta_r_error;
    dtheta_r_error_v(i) = dtheta_r_error;
    
    
    tau_p_v(i) = tau_p;
    theta_p_error_v(i) = theta_p_error;
    dtheta_p_error_v(i) = dtheta_p_error;
    
    
    tau_y_v(i) = tau_y;
    theta_y_error_v(i) = theta_y_error;
    dtheta_y_error_v(i) = dtheta_y_error;
    
    %% Velocities of the rotors
    [omega_square]=vel_rotors_f(T,tau_r,tau_p,tau_y,quad);
    
    %% Jacobian matrix
    [J]=jacobian_matrix_f(theta_r, theta_p, theta_y,quad);
    
    %% C matrix
    [C]=matriz_C_f(theta_r,theta_p,theta_y, dtheta_r,dtheta_p,dtheta_y,quad);
    
    %% Angular acceleration in the inertial frame
    [d2n]=angular_acceleration_inertial_frame_f(J,tau_B, C,dtheta_r,dtheta_p,dtheta_y);
    d2theta_r=d2n(1);
    d2theta_p=d2n(2);
    d2theta_y=d2n(3);
    
    %% Angular velocity in the inertial frame
    dn=d2n*dt+dn_ant;
    dn_ant=dn;
    
    dtheta_r=dn(1);
    dtheta_p=dn(2);
    dtheta_y=dn(3);
    
    %%  Roll, pitch and yaw angles
    n=dn*dt+n_ant;
    n_ant=n;
    
    theta_r=normalize_angle_f(n(1),-pi);
    theta_p=normalize_angle_f(n(2),-pi);
    theta_y=normalize_angle_f(n(3),-pi);
    
    theta_r_v(i) = theta_r*180/pi;
    theta_p_v(i) = theta_p*180/pi;
    theta_y_v(i) = theta_y*180/pi;
    
    %% Linear acceleration in the inertial frame
    [d2s]=linear_acceleration_inertial_frame_f(dx,dy,dz,T,theta_r,theta_p,theta_y,quad);
    d2x=d2s(1);
    d2y=d2s(2);
    d2z=d2s(3);
    
    %% Linear velocity in the inertial frame
    
    ds=d2s*dt+ds_ant;
    ds_ant=ds;
    
    dx=ds(1);
    dy=ds(2);
    dz=ds(3);
    
    %% Position in the inertial frame
    
    s=ds*dt+s_ant;
    s_ant=s;
    
    x=s(1);
    y=s(2);
    z=s(3);
    
    x_v(i) = x;
    y_v(i) = y;
    z_v(i) = z;
    
    %% Displaces the quadrotor object
    pause(0.01)
    set(gcf,'CurrentAxes',gui.axes1)
    %  move_object_f(rmr_obj,[theta_r;theta_p;theta_y],[x;y;z]);
    move_object_f(quad1_obj,[theta_r;theta_p;theta_y],[x;y;z]);
    drawnow
    set(gcf,'CurrentAxes',gui.axes2)
    %  move_object_f(rmr_obj,[theta_r;theta_p;theta_y],[x;y;z]);
    move_object_f(quad2_obj,[theta_r;theta_p;theta_y],[x;y;z]);
    drawnow

    set(gui.edit_time,'string',t)
    
    set(gui.edit_posx,'string',x)
    set(gui.edit_posy,'string',y)
    set(gui.edit_posz,'string',z)
    
    set(gui.edit_roll,'string',theta_r*180/pi)
    set(gui.edit_pitch,'string',theta_p*180/pi)
    set(gui.edit_yaw,'string',theta_y*180/pi)
    
    %fprintf('x = %f, y =%f, z =%f, yaw = %f \n',x,y,z,theta_y*180/pi)
    
   
end
