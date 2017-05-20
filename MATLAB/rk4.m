function XU = rk4(F,X,V,U,C,dt)
% F = State Model
% X = Current State Vector
% V = Process Noise
% U = Control Vector
% C = Robot Constants
% dt = Integration Time

k1 = F(X,V,U,C);
k2 = F(X+dt/2*k1,V,U,C);
k3 = F(X+dt/2*k2,V,U,C);
k4 = F(X+dt*k3,V,U,C);
XU = X + dt/6*(k1 + 2*k2 + 2*k3 + k4);