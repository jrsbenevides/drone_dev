function [Krlqr,F_inc,G_inc,w] = loop(F,G,R,Q)
persistent P

if isempty(P)
    P = eye(4);
end
w = normrnd(0,1);
mi = 1e20;
delta = 0*unifrnd(-1,1); %p x l
H = ones(4,1); %n x p
Ef = 1*[0.9 0.5 0.9 0.6]; %1 x n
Eg = 1*[0.81 0.9 0.7 0.9]; %1 x m

lambda = norm(mi*H'*H) + 0.01;

sigma = [(mi^-1)*eye(4)-(lambda^-1)*H*H' zeros(4,1); zeros(1,4) (eye(1)*lambda^-1)];
F_inc = F + H*delta*Ef;
G_inc = G + H*delta*Eg;


n = size(F,1);
m = size(G,2);
l = size(Ef,1);
t = n+1;%+2;

F_est = [F; Ef];
G_est = [G; Eg];
I_est = [eye(n); zeros(l,n)];


V = [ zeros(n,n)  zeros(n,m)  zeros(n,n)
      zeros(m,n)  zeros(m,m)  zeros(m,n)
      zeros(n,n)  zeros(n,m)    -eye(n)
      zeros(t,n)  zeros(t,m)    F_est
        eye(n)    zeros(n,m)  zeros(n,n)
      zeros(m,n)    eye(m)    zeros(m,n) ];
  
Ur = [ zeros(n,n) ; zeros(m,n) ; -eye(n) ; F_est ; zeros(n,n) ; zeros(m,n)];

M = [    inv(P)    zeros(n,m) zeros(n,n) zeros(n,t)   eye(n)   zeros(n,m)
       zeros(m,n)   inv(R)    zeros(m,n) zeros(m,t) zeros(m,n)   eye(m)
       zeros(n,n)  zeros(n,m)   inv(Q)   zeros(n,t) zeros(n,n) zeros(n,m)
       zeros(t,n)  zeros(t,m) zeros(t,n)   sigma      I_est     -G_est
         eye(n)    zeros(n,m) zeros(n,n)   I_est'   zeros(n,n) zeros(n,m)
       zeros(m,n)    eye(m)   zeros(m,n)  -G_est'   zeros(m,n) zeros(m,m)];
 
M_cal = V'*(M\Ur); 
   
L = zeros(n);
Krlqr = zeros(m,n);
L(:,:) = M_cal(1:n,1:n);
Krlqr(:,:) = M_cal(n+1:n+m,1:n);
P(:,:) = M_cal(n+m+1:2*n+m,1:n);


end
   
   
