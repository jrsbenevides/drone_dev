function [x_local] = globalToLocal(x_global,psi)

x_local = [cos(psi) sin(psi) 0 0;-sin(psi) cos(psi) 0 0;0 0 1 0;0 0 0 1]*x_global;

end
