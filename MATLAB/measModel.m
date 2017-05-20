function Y = measModel(X,N,C,D,measconfig)
% X = State Vector
%   x       - Position                              [m]
%   y       - Position                              [m]
%   theta   - Heading                               [rad]
%   vt      - Forward Velocity                      [m/s]
%   wt      - Angular Velocity                      [rad/s]
% N = Measurement Noise
%   n1      - Measurement Noise (Right Distance)    [m]
%   n2      - Measurement Noise (Left Distance)     [m]
%   n3      - Measurement Noise (Right Wheel)       [rad/s]
%   n4      - Measurement Noise (Left Wheel)        [rad/s]
% C = Robot Constants
%   a       - 1/Motor Time Constant                 [1/s]
%   b       - Volts To Angular Acceleration         [rad/s^2/V]
%   r       - Wheel Radius                          [m]
%   g       - Gear Ratio
%   m       - Distance From Wheel Base To CG        [m]
%   n       - Distance Between Wheels               [m]
%   p       - Distance From CG To Distance Sensors  [m]
%   q       - Distance Between Distance Sensors     [m]
% D = Path Parameters
%   Straightaway
%       xr
%       yr
%       thetar
%       s
%   Turn
%       xc
%       yc
%       R
%       s
% measconfig = Measurement Type
%   0 - Side Wall Distance
%   1 - Angular Velocity
%
% Y = Expected Measurements
%   Side Wall Distance
%       dR  - Distance To Right Wall
%       dL  - Distance To Left Wall
%   Angular Velocity
%       wR  - Right Wheel Angular Velocity
%       wL  - Left Wheel Angular Velocity

l = 0.18; % Distance Between Walls [m]

L = size(X,2); % Number Of Sigma Points
Y = zeros(2,L);

x = X(1,:);
y = X(2,:);
theta = X(3,:);
vt = X(4,:);
wt = X(5,:);

switch measconfig
    case 0
        n1 = N(1,:);
        n2 = N(2,:);
        
        p = C(7);
        q = C(8);
        
        xr = D(1);
        yr = D(2);
        thetar = D(3);
        
        e = (y-yr)*cos(thetar) - (x-xr)*sin(thetar);
        ep = e + p*sin(theta-thetar);
        
        % Right Distance [m]
        Y(1,:) = (ep+l/2).*sec(theta-thetar) - q/2.*ones(1,L) + n1;
        %Left Distance [m]
        Y(2,:) = (l/2-ep).*sec(theta-thetar) - q/2.*ones(1,L) + n2;
    case 1
        n1 = N(3,:);
        n2 = N(4,:);
        
        r = C(3);
        g = C(4);
        n = C(6);
        
        Y(1,:) = (vt + n/2*wt)/r/g + n1; % Angular Velocity Right Motor
        Y(2,:) = (vt - n/2*wt)/r/g + n2; % Anuglar Velocity Left Motor
    otherwise
end