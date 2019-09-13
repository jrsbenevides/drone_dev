clear all
close all
clc

%Carrega os K's

load('constantesEstimadas.mat');

%[X,dX,d2X] = quadModel_f(K,U,X,dX,dt)

%Condições Iniciais
    
    % Feedback_Lin (TC)
    X   = [0 0 0 0]';
    dX  = [0 0 0 0]';
    d2X = [0 0 0 0]';
    
    % PID
    Xpid   = [0 0 0 0]';
    dXpid  = [0 0 0 0]';
    d2Xpid = [0 0 0 0]';

    var = zeros(4,1);
    varRF = zeros(4,1);
    
    % RLQR
    XRLQR   = [0 0 0 0]';
    dXRLQR  = [0 0 0 0]';
    d2XRLQR = [0 0 0 0]';

    Q = 5000*[1 0 0 0
         0 1 0 0
         0 0 1 0
         0 0 0 1];
    R = 1*[1 0 0 0
         0 1 0 0
         0 0 1 0
         0 0 0 1]; 
     
    % RLQR + FL
    XRF   = [0 0 0 0]';
    dXRF  = [0 0 0 0]';
    d2XRF = [0 0 0 0]';

    QRF = 1e1*[1 0 0 0
         0 1 0 0
         0 0 1 0
         0 0 0 100];
    RRF = 1e1*[1 0 0 0
         0 2 0 0
         0 0 3 0
         0 0 0 100]; 
     
   % LQR
    XLQR   = [0 0 0 0]';
    dXLQR  = [0 0 0 0]';
    d2XLQR = [0 0 0 0]';

    Ql = 1e2*[1 0 0 0
         0 3 0 0
         0 0 5 0
         0 0 0 100];
    Rl = 1e5*[1 0 0 0
         0 7 0 0
         0 0 3 0
         0 0 0 100];
     
    N = zeros(4,4);
    % LQRf
    XLQRf   = [0 0 0 0]';
    dXLQRf  = [0 0 0 0]';
    d2XLQRf = [0 0 0 0]';

    Qlqrf = 1e10*[1 0 0 0
         0 1 0 0
         0 0 1 0
         0 0 0 100];
    Rlqrf = 1e9*[1 0 0 0
         0 1 0 0
         0 0 1 0
         0 0 0 100];
     
    Nf = zeros(4,4);

    % Dados
    Dados(1,1) = 0;
    Dados(1,2) = 0;
    Dados(1,3) = 0;
    Dados(1,4) = 0;
    Dados(1,5) = 0;
    Dados(1,6) = 0;
    Dados(1,7) = 0;
    Dados(1,8) = 0;
    Dados(1,9) = 0;
    
%Referencia
Xr   = [1 1 1 1]';
dXr  = [0 0 0 0]';
d2Xr = [0 0 0 0]';

%Ganhos

% FeedBack Lin
kp = 80*diag([1 1 1 1]);
kd = 10*diag([1 1 1 1]);

% PID
kpPID = 350*diag([10 10 10 10]);
kdPID = 95*diag([10 10 10 10]);
kiPID = 40*diag([10 10 10 10]);

freq = 100;
dt = 1/freq;


%figure(1)

t = 1:1001;

for i = 1:1000
    
    %% Feedback Linearization
    Xe(1:3,1) = Xr(1:3,1) - X(1:3,1);
    Xe(4,1) = normalize_angle_f(Xr(4,1) - X(4,1),-pi);
    dXe = dXr - dX;
    
    dXb = globalToLocal(dX,dX(4));
    
    V = d2Xr + kp*Xe + kd*dXe;
    
    [f1,f2] = quadModel(K,X);
    
        % Adicionando Incerteza
        [Krlqr2,F_inc2,G_inc2] = loop(f2,f1,R,Q);
        
    % Computação da ação de controle
    U = inv(f1)*(V + f2*dXb);
     
    for j = 1:size(U,1)
        if U(j,:) > 1
            U(j,:) = 1;
        end
        if U(j,:) < -1
            U(j,:) = -1;
        end
    end

    psi = X(4);

    dXb = globalToLocal(dX,psi);     

    d2X = G_inc2*U - F_inc2*dXb;

    dX = d2X*dt + dX;

    X = dX*dt + X;

%     [X,dX,d2X,dXb] = quadStates(K,U,X,dX,dt);
   
    %% PID
    Xepid(1:3,1) = Xr(1:3,1) - Xpid(1:3,1);
    Xepid(4,1) = normalize_angle_f(Xr(4,1) - Xpid(4,1),-pi);
    dXepid = dXr - dXpid;
    
    dXbpid = globalToLocal(dXpid,dXpid(4));
    % Integração do erro
    IXe = var + Xepid*dt;
    var = IXe;
    % Modelo Nominal
    [f1,f2] = quadModel(K,Xpid);
        % Adicionando Incerteza
        [Krlqr3,F_inc3,G_inc3] = loop(f2,f1,R,Q);
    % Computação da ação de controle
    Upid = kpPID*Xepid + kdPID*dXepid + kiPID*IXe;
    for j = 1:size(Upid,1)
        if Upid(j,:) > 1
            Upid(j,:) = 1;
        end
        if Upid(j,:) < -1
            Upid(j,:) = -1;
        end
    end
    % Obtenção dos Estados
    psi = Xpid(4);
    dXbpid = globalToLocal(dXpid,psi);     
    d2Xpid = G_inc3*Upid - F_inc3*dXbpid;
    dXpid = d2Xpid*dt + dXpid;
    Xpid = dXpid*dt + Xpid;
%   [Xpid,dXpid,d2Xpid,dXbpid] = quadStates(K,Upid,Xpid,dXpid,dt);
 
    %% RLQR
    
    XeRLQR(1:3,1) = (Xr(1:3,1) - XRLQR(1:3,1));
    XeRLQR(4,1) = (normalize_angle_f(Xr(4,1) - XRLQR(4,1),-pi));
    dXeRLQR = (dXr - dXRLQR);
    
    dXbRLQR = globalToLocal(dXRLQR,dXRLQR(4));
    
    %   V = d2Xr + kp*Xe + kd*dXe;
    %   U = inv(f1)*(V + f2*dXb);
    
    [f1,f2] = quadModel(K,XRLQR);
        
        % Adicionando Robustez
        [Krlqr,F_inc,G_inc] = loop(f2,f1,R,Q);

    URLQR = -Krlqr*XeRLQR;
    
    u = inv(f1)*(URLQR + d2Xr + f2*dXr);
    
    for j = 1:size(u,1)
        if u(j,:) > 1
            u(j,:) = 1;
        end
        if u(j,:) < -1
            u(j,:) = -1;
        end
    end
    DadoU{i+1} = URLQR;
    
%     d2XeRLQR = -F_inc*dXebRLQR + G_inc*URLQR;
%     dXeRLQR = d2XeRLQR*dt + dXeRLQR;
%     XeRLQR = dXeRLQR*dt + XeRLQR;    
%     XRLQR = XeRLQR + Xr;
    
    % QuadStates para o caso Robusto
    psi = XRLQR(4);
 
    dXbRLQR = globalToLocal(dXRLQR,psi);     

    d2XRLQR = G_inc*u - F_inc*dXbRLQR;

    dXRLQR = d2XRLQR*dt + dXRLQR;

    XRLQR = dXRLQR*dt + XRLQR;
    
    %[XRLQR,dXRLQR,d2RLQR,dXRLQR] = quadStates(K,URLQR,XRLQR,dXRLQR,dt);
    
    %% PID + Feedback Linearization
    XeRF(1:3,1) = Xr(1:3,1) - XRF(1:3,1);
    XeRF(4,1) = normalize_angle_f(Xr(4,1) - XRF(4,1),-pi);
    dXeRF = dXr - dXRF;
    
    dXbRF = globalToLocal(dXRF,dXRF(4));
    
    % Integração do erro
    IXeRF = varRF + XeRF*dt;
    varRF = IXeRF;
    
    [f1,f2] = quadModel(K,XRF);
    
        % Adicionando Incerteza
        [KRF,F_incRF,G_incRF] = loop(f2,f1,RRF,QRF);
    
    UpidRF = kpPID*XeRF + kdPID*dXeRF + kiPID*IXeRF;
    
    VRF = d2Xr + UpidRF;
    
    % Computação da ação de controle
    uRF = inv(f1)*(VRF + f2*dXbRF);
     
    for j = 1:size(uRF,1)
        if uRF(j,:) > 1
            uRF(j,:) = 1;
        end
        if uRF(j,:) < -1
            uRF(j,:) = -1;
        end
    end

    psi = XRF(4);

    dXbRF = globalToLocal(dXRF,psi);     

    d2XRF = G_incRF*uRF - F_incRF*dXb;

    dXRF = d2XRF*dt + dXRF;

    XRF = dXRF*dt + XRF;

%     [X,dX,d2X,dXb] = quadStates(K,U,X,dX,dt);
    %% LQR
    
    XeLQR(1:3,1) = (Xr(1:3,1) - XLQR(1:3,1));
    XeLQR(4,1) = (normalize_angle_f(Xr(4,1) - XLQR(4,1),-pi));
    dXeLQR = (dXr - dXLQR);
    
    dXbLQR = globalToLocal(dXLQR,dXLQR(4));
    
    %   V = d2Xr + kp*Xe + kd*dXe;
    %   U = inv(f1)*(V + f2*dXb);
    
    [f1,f2] = quadModel(K,XLQR);
    
    sys = ss(-f2,f1,zeros(4,4),zeros(4,4));
    
        % Fun��o RQL (Willian)
        %[L,Klqr,Plqr] = robust_control(f2,f1,Qlqr,Rlqr);
        [Klqr,S,e] = lqr(sys,Ql,Rl,N); 
        
        % Adicionando Incerteza
        [k,F_lqr,G_lqr] = loop(f2,f1,Rl,Ql);
        clear k
        clear e
        
    ULQR = -Klqr*XeLQR;
    
    ulqr = inv(f1)*(ULQR + d2Xr + f2*dXr);
    
    for j = 1:size(ulqr,1)
        if ulqr(j,:) > 1
            ulqr(j,:) = 1;
        end
        if ulqr(j,:) < -1
            ulqr(j,:) = -1;
        end
    end
    
%     d2XeRLQR = -F_inc*dXebRLQR + G_inc*URLQR;
%     dXeRLQR = d2XeRLQR*dt + dXeRLQR;
%     XeRLQR = dXeRLQR*dt + XeRLQR;    
%     XRLQR = XeRLQR + Xr;
    
    % QuadStates para o caso Robusto
    psi = XLQR(4);
 
    dXbLQR = globalToLocal(dXLQR,psi);     

    d2XLQR = G_lqr*ulqr - F_lqr*dXbLQR;

    dXLQR = d2XLQR*dt + dXLQR;

    XLQR = dXLQR*dt + XLQR;
    
    %[XRLQR,dXRLQR,d2RLQR,dXRLQR] = quadStates(K,URLQR,XRLQR,dXRLQR,dt);

    %% LQR + FL
    
    XeLQRf(1:3,1) = (Xr(1:3,1) - XLQRf(1:3,1));
    XeLQRf(4,1) = (normalize_angle_f(Xr(4,1) - XLQRf(4,1),-pi));
    dXeLQRf = (dXr - dXLQRf);
    
    dXbLQRf = globalToLocal(dXLQRf,dXLQRf(4));
    
    %   V = d2Xr + kp*Xe + kd*dXe;
    %   U = inv(f1)*(V + f2*dXb);
    
    [f1,f2] = quadModel(K,XLQRf);
    
    sysf = ss(-f2,f1,zeros(4,4),zeros(4,4));
    
        % Fun��o RQL (Willian)
        %[L,Klqr,Plqr] = robust_control(f2,f1,Qlqr,Rlqr);
        [Klqrf,S,e] = lqr(sysf,Q,R,N); 
        
        % Adicionando Incerteza
        [k,F_lqrf,G_lqrf] = loop(f2,f1,R,Q);
        clear k
        
    ULQRf = -Klqrf*XeLQRf;
    
    Vlqrf = d2Xr + ULQRf;
    
    ulqrf = inv(f1)*(Vlqrf + f2*dXbLQRf);
    
    for j = 1:size(ulqrf,1)
        if ulqrf(j,:) > 1
            ulqrf(j,:) = 1;
        end
        if ulqrf(j,:) < -1
            ulqrf(j,:) = -1;
        end
    end
    
%     d2XeRLQR = -F_inc*dXebRLQR + G_inc*URLQR;
%     dXeRLQR = d2XeRLQR*dt + dXeRLQR;
%     XeRLQR = dXeRLQR*dt + XeRLQR;    
%     XRLQR = XeRLQR + Xr;
    
    % QuadStates para o caso Robusto
    psi = XLQRf(4);
 
    dXbLQRf = globalToLocal(dXLQRf,psi);     

    d2XLQRf = G_lqrf*ulqrf - F_lqrf*dXbLQRf;

    dXLQRf = d2XLQRf*dt + dXLQRf;

    XLQRf = dXLQRf*dt + XLQRf;
    
    %[XRLQR,dXRLQR,d2RLQR,dXRLQR] = quadStates(K,URLQR,XRLQR,dXRLQR,dt);
    
    %% Dados
    
    Dados(i+1,1) = X(1);
    Dados(i+1,2) = X(2);
    Dados(i+1,3) = X(3);
    Dados(i+1,4) = XRLQR(1);
    Dados(i+1,5) = XRLQR(2);
    Dados(i+1,6) = XRLQR(3);
    Dados(i+1,7) = Xpid(1);
    Dados(i+1,8) = Xpid(2);
    Dados(i+1,9) = Xpid(3);
    Dados(i+1,10) = XRF(1);
    Dados(i+1,11) = XRF(2);
    Dados(i+1,12) = XRF(3);
    Dados(i+1,13) = XLQR(1);
    Dados(i+1,14) = XLQR(2);
    Dados(i+1,15) = XLQR(3);
    Dados(i+1,16) = XLQRf(1);
    Dados(i+1,17) = XLQRf(2);
    Dados(i+1,18) = XLQRf(3);
    
    
    %% Plot Animado
    
    
%     subplot(3,1,1);
%     plot(t(i),X(1),'.b')
%     hold on;
%     %plot(t(i),Xpid(1),'.r')
%     plot(t(i),XRLQR(1),'g')
%     legend('Feed Lin','PID', 'RLQR')
%     drawnow;
%     subplot(3,1,2);
%     plot(t(i),X(2),'.b')
%     hold on;
%     %plot(t(i),Xpid(2),'.r')
%     plot(t(i),XRLQR(2),'.g')
%     subplot(3,1,3);
%     plot(t(i),X(3),'.b')
%     hold on;
%     plot(t(i),Xpid(3),'.r')
%     plot(t(i),XRLQR(3),'.g')
%     subplot(4,1,4);
%     plot(t(i),X(4),'.b')
%     hold on;
%     plot(t(i),Xpid(4),'.r')
%     plot(t(i),XRLQR(4),'.g')
%     fprintf('%.1f\n',i*100/1000);

    
    
end

DadoU = cell2mat(DadoU);

    figure
    subplot(2,1,1);
    plot(t,Dados(:,1),'b')
    hold on;
    plot(t,Dados(:,4),'r')
    %plot(t,Dados(:,7),'g')
    plot(t,Dados(:,10),'y')
    plot(t,Dados(:,13),'m')
    plot(t,Dados(:,16),'k')
    legend('Feed Lin','RLQR','PID+FL','LQR','LQR+FL')
    title('X')
    subplot(2,1,2);
    plot(t,Dados(:,2),'b')
    hold on;
    plot(t,Dados(:,5),'r')
    %plot(t,Dados(:,8),'g')
    plot(t,Dados(:,11),'y')
    plot(t,Dados(:,14),'m')
    plot(t,Dados(:,17),'k')
    title('Y')
    
%     figure
%     subplot(3,1,1);
%     plot(t,Dados(:,1),'b')
%     hold on;
%     plot(t,Dados(:,7),'r')
%     plot(t,Dados(:,4),'g')
%     legend('Feed Lin','PID','RLQR')
%     drawnow;
%     subplot(3,1,2);
%     plot(t,Dados(:,2),'b')
%     hold on;
%     plot(t,Dados(:,8),'r')
%     plot(t,Dados(:,5),'g')
%     subplot(3,1,3);
%     plot(t,Dados(:,3),'b')
%     hold on;
%     plot(t,Dados(:,9),'r')
%     plot(t,Dados(:,6),'g')
    
