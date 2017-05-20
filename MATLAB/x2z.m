function [Z,alpha,rho] = x2z(X,C,D,pathconfig)
% X = State Vector
%   x       - Position                              [m]
%   y       - Position                              [m]
%   theta   - Heading                               [rad]
%   vt      - Forward Velocity                      [m/s]
%   wt      - Angular Velocity                      [rad/s]
% C = Robot Constants
%   a       - 1/Motor Time Constant                 [1/s]
%   b       - Volts To Angular Acceleration         [rad/s^2/V]
%   r       - Wheel Radius                          [m]
%   g       - Gear Ratio
%   m       - Distance From Wheel Base To CG        [m]
%   n       - Distance Between Wheels               [m]
%   p       - Distance From CG To Distance Sensors  [m]
%   q       - Distance Between Distance Sensors     [m]
% D = Path Constraints
%   Straightaway
%       xr      - Reference X Position              [m]
%       yr      - Reference Y Position              [m]
%       thetar  - Reference Heading                 [rad]
%       s       - Desired Speed                     [m/s]
%   Turns
%       xc      - Center Of Curvature, X            [m]
%       yc      - Center Of Curvature, Y            [m]
%       R       - Radius Of Curvature               [m]
%       s       - Desired Speed                     [m/s]
% pathconfig = Control Type
%   0 - Straightaway
%   1 - Turn

x = X(1);
y = X(2);
theta = X(3);
vt = X(4);
wt = X(5);

a = C(1);
b = C(2);
r = C(3);
g = C(4);
m = C(5);
n = C(6);

switch pathconfig
    case 0
        % Straightaway
        xr = D(1);
        yr = D(2);
        thetar = D(3);
        s = D(4);
        
        z1 = (y-yr)*cos(thetar) - (x-xr)*sin(thetar);
        z2 = vt*sin(theta-thetar) + m*wt*cos(theta-thetar);
        z3 = vt^2 + (m*wt)^2 - s^2;
        
        Z = [z1; z2; z3];
        
        alpha = [(-a*vt-m*wt^2)*sin(theta-thetar) + ...
            (-a*m*wt+vt*wt)*cos(theta-thetar);
            -2*a*(vt^2+m*wt^2)];
        
        rho = [b*g*r/2*sin(theta-thetar), b*g*r/n*m*cos(theta-thetar)
            b*g*r*vt, 2*b*g*r/n*m*wt];
    case 1
        % Turn
        xc = D(1);
        yc = D(2);
        R = D(3);
        s = D(4);
        
        z1 = (x-xc)^2 + (y-yc)^2 - R^2;
        z2 = 2*(x-xc)*(vt*cos(theta)-m*wt*sin(theta)) + ...
            2*(y-yc)*(vt*sin(theta)+m*wt*cos(theta));
        z3 = vt^2 + (m*wt)^2 - s^2;
        
        Z = [z1; z2; z3];
        
        alpha = [2*(vt^2+(m*wt)^2) + ...
            2*(x-xc)*((-a*vt-m*wt^2)*cos(theta)+(-vt*wt+a*m*wt)*sin(theta)) + ...
            2*(y-yc)*((-a*vt-m*wt^2)*sin(theta)+(vt*wt-a*m*wt)*cos(theta));
            -2*a*(vt^2+m*wt^2)];
        
        rho = [(x-xc)*b*g*r*cos(theta)+(y-yc)*b*g*r*sin(theta), ...
            -2*(x-xc)*b*g*r/n*m*sin(theta)+2*(y-yc)*b*g*r/n*m*cos(theta)
            b*g*r*vt, 2*b*g*r/n*m*wt];
    otherwise
end