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
    
    % RLQR
    
    XRLQR   = [0 0 0 0]';
    dXRLQR  = [0 0 0 0]';
    d2XRLQR = [0 0 0 0]';

    Q = 1e10*[1 0 0 0
         0 1 0 0
         0 0 1 0
         0 0 0 100];
    R = 1e9*[1 0 0 0
         0 1 0 0
         0 0 1 0
         0 0 0 100]; 

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
Xr   = [1 1 1 0]';
dXr  = [0 0 0 0]';
d2Xr = [0 0 0 0]';

%Ganhos

% FeedBack Lin
kp = 10*diag([1 1 1 1]);
kd = 10*diag([1 1 1 1]);

% PID
kpPID = 300*diag([10 10 10 10]);
kdPID = 90*diag([10 10 10 10]);
kiPID = 30*diag([10 10 10 10]);

freq = 100;
dt = 1/freq;


%figure(1)

t = 1:2001;

for i = 1:2000
    
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
    IXe = var + Xepid*dt
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
    
%     [Xpid,dXpid,d2Xpid,dXbpid] = quadStates(K,Upid,Xpid,dXpid,dt);
    
    %% RLQR
    
    XeRLQR(1:3,1) = (Xr(1:3,1) - XRLQR(1:3,1));
    XeRLQR(4,1) = (normalize_angle_f(Xr(4,1) - XRLQR(4,1),-pi));
    dXeRLQR = (dXr - dXRLQR);
    
    dXbRLQR = globalToLocal(dXRLQR,dXRLQR(4));
    
    %   V = d2Xr + kp*Xe + kd*dXe;
    %   U = inv(f1)*(V + f2*dXb);
    
    [f1,f2] = quadModel(K,XRLQR);
    
    [Krlqr,F_inc,G_inc] = loop(f2,f1,R,Q);
    
    URLQR = -Krlqr*XeRLQR;
    
    for j = 1:size(URLQR,1)
        if URLQR(j,:) > 1
            URLQR(j,:) = 1;
        end
        if URLQR(j,:) < -1
            URLQR(j,:) = -1;
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

    d2XRLQR = G_inc*URLQR - F_inc*dXbRLQR;

    dXRLQR = d2XRLQR*dt + dXRLQR;

    XRLQR = dXRLQR*dt + XRLQR;
    
    %[XRLQR,dXRLQR,d2RLQR,dXRLQR] = quadStates(K,URLQR,XRLQR,dXRLQR,dt);
 
    %% RLQR2
    
%     XeRLQR2(1:3,1) = (Xr(1:3,1) - XRLQR2(1:3,1));
%     XeRLQR2(4,1) = (normalize_angle_f(Xr(4,1) - XRLQR2(4,1),-pi));
%     dXeRLQR2 = (dXr - dXRLQR2);
%     
%     dXeRLQR2 = globalToLocal(dXRLQR,dXRLQR(4));
%     
%     %   V = d2Xr + kp*Xe + kd*dXe;
%     %   U = inv(f1)*(V + f2*dXb);
%     
%     [f1,f2] = quadModel(K,XRLQR);
%     
%     [Krlqr,F_inc,G_inc] = loop(f2,f1,R,Q);
%     
%     URLQR = -Krlqr*XeRLQR;
%     
%     for j = 1:size(URLQR,1)
%         if URLQR(j,:) > 1
%             URLQR(j,:) = 1;
%         end
%         if URLQR(j,:) < -1
%             URLQR(j,:) = -1;
%         end
%     end
%     DadoU{i+1} = URLQR;
%     
% %     d2XeRLQR = -F_inc*dXebRLQR + G_inc*URLQR;
% %     dXeRLQR = d2XeRLQR*dt + dXeRLQR;
% %     XeRLQR = dXeRLQR*dt + XeRLQR;    
% %     XRLQR = XeRLQR + Xr;
%     
%     % QuadStates para o caso Robusto
%     psi = XRLQR(4);
%  
%     dXbRLQR = globalToLocal(dXRLQR,psi);     
% 
%     d2XRLQR = G_inc*URLQR - F_inc*dXbRLQR;
% 
%     dXRLQR = d2XRLQR*dt + dXRLQR;
% 
%     XRLQR = dXRLQR*dt + XRLQR;
%     
%     %[XRLQR,dXRLQR,d2RLQR,dXRLQR] = quadStates(K,URLQR,XRLQR,dXRLQR,dt);
    
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
    plot(t,Dados(:,7),'g')
    legend('Feed Lin','RLQR','PID')
    title('X')
    subplot(2,1,2);
    plot(t,Dados(:,2),'b')
    hold on;
    plot(t,Dados(:,5),'r')
    plot(t,Dados(:,8),'g')
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
    
