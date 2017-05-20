function Xdot = stateModel(X,V,U,C)
% X = State Vector (Can Be A Matrix Of Column Vectors)
%   x       - Position                              [m]
%   y       - Position                              [m]
%   theta   - Heading                               [rad]
%   vt      - Forward Velocity                      [m/s]
%   wt      - Angular Velocity                      [rad/s]
% V = Process Noise (Can Be A Matrix Of Column Vectors)
%   v1      - Process Noise Right Wheel             [V]
%   v2      - Process Noise Left Wheel              [V]
% U = Control Vector (Always A Column Vector)
%   u1      - Sum Of Applied Voltage To Both Wheels [V]
%   u2      - Difference Between Applied Voltage    [V]
% C = Robot Constants (Always A Column Vector)
%   a       - 1/Motor Time Constant                 [1/s]
%   b       - Volts To Angular Acceleration         [rad/s^2/V]
%   r       - Wheel Radius                          [m]
%   g       - Gear Ratio
%   m       - Distance From Wheel Base To CG        [m]
%   n       - Distance Between Wheels               [m]
%   p       - Distance From CG To Distance Sensors  [m]
%   q       - Distance Between Distance Sensors     [m]

L = size(X,2); % Number Of Sigma Points

theta = X(3,:); 
vt = X(4,:); 
wt = X(5,:);

v1 = V(1,:); 
v2 = V(2,:);

u1 = U(1); 
u2 = U(2);

a = C(1); 
b = C(2);
r = C(3); 
g = C(4); 
m = C(5);
n = C(6);

xdot = vt.*cos(theta) - m*wt.*sin(theta);
ydot = vt.*sin(theta) + m*wt.*cos(theta);
thetadot = wt;
vtdot = -a.*vt + b*g*r/2*(u1*ones(1,L) + v1);
wtdot = -a.*wt + b*g*r/n*(u2*ones(1,L) + v2);

Xdot = [xdot; ydot; thetadot; vtdot; wtdot];