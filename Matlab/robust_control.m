function [L,Klqr,Plqr] = robust_control(R,Q,f2,f1)

Plqr = eye(4);

Ef = 0*[1.9 1.5 1.9 1.6]; %1 x n
Eg = 0*[1.81 1.9 1.7 1.9]; %1 x m

[nx,nu]  = size(f1); 
 nr      = size(Eg,1);

I_r = [eye(nx,nx); zeros(nr,nx)];

G_r = [f1; Eg];
F_r = [f2; Ef];

X  = [ Plqr^(-1)          zeros(nx,nu)    zeros(nx,nx)    zeros(nx,nr+nx)    eye(nx,nx)   zeros(nx,nu)
       zeros(nu,nx)    R^(-1)          zeros(nu,nx)    zeros(nu,nr+nx)    zeros(nu,nx) eye(nu,nu)
       zeros(nx,nx)    zeros(nx,nu)    Q^(-1)          zeros(nx,nr+nx)    zeros(nx,nx) zeros(nx,nu)
       zeros(nr+nx,nx) zeros(nr+nx,nu) zeros(nr+nx,nx) zeros(nr+nx,nr+nx) I_r          -G_r
       eye(nx,nx)      zeros(nx,nu)    zeros(nx,nx)    I_r'               zeros(nx,nx) zeros(nx,nu)
       zeros(nu,nx)    eye(nu,nu)      zeros(nu,nx)   -G_r'               zeros(nu,nx) zeros(nu,nu)];

   
Zeta = [zeros(nx,nx)    zeros(nx,nu)     zeros(nx,nx);
        zeros(nu,nx)    zeros(nu,nu)     zeros(nu,nx);
        zeros(nx,nx)    zeros(nx,nu)    -eye(nx,nx);
        zeros(nx+nr,nx) zeros(nx+nr,nu)  F_r;
        eye(nx,nx)      zeros(nx,nu)     zeros(nx,nx);
        zeros(nu,nx)    eye(nu,nu)       zeros(nu,nx)];

Xi = [zeros(nx,nx); zeros(nu,nx); -eye(nx,nx); F_r; zeros(nx,nx); zeros(nu,nx)];
       
       
LKP = Zeta'*X^(-1)*Xi;

L = LKP(1:nx,:);
Klqr = LKP(nx+1:nx+nu,:);
Plqr = LKP(nx+nu+1:end,:);




