function [X,dX,d2X,dXb] = quadStates(K,U,X,dX,dt)

[f1,f2] = quadModel(K,X);

psi = X(4);

dXb = globalToLocal(dX,psi);     
      
d2X = f1*U - f2*dXb;
      
dX = d2X*dt + dX;

X = dX*dt + X;

end