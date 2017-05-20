function [X,Y,YD,Yconfig,T,U] = runSim(X0,N,C,D,pathconfig,pathtest)
% Inputs:
%   X0          - Initial State Vector
%   N           - Measurement Noise Standard Deviation
%   C           - Robot Constants
%   D           - Path Parameters
%   pathconfig  - Type Of Path
%   pathtest    - End Of Path Condition
% Outputs:
%   X           - State Vector
%   Y           - Measurement Vector
%   T           - Time Vector
%   U           - Control Vector

% Initialize Variables
X = X0;
Y = [];
YD = [];
Yconfig = [];
T = 0;
U = [];

% Clamp theta, -pi < theta < pi
while abs(X(3)) > pi
    if X(3) > pi
        X(3) = X(3) - 2*pi;
    elseif X(3) < -pi
        X(3) = X(3) + 2*pi;
    end
end

% Compute Control Gain
A = [0, 1, 0; 0, 0, 0; 0, 0, 0];
B = [0, 0; 1, 0; 0, 1];
Q = diag([1000, 1000, 100]);
R = diag([1, 1]);
K = computeControlGain(A,B,Q,R);

UMAX = 10; % Max Voltage To Motors

% Simulation Parameters
% For Convenience I'm Pretending That Everything Happens On The Same
% Frequency.
freq = 50; % [Hz]
dt = 1/freq/2;
count = 0;

% Compute First Control Command
[Z,alpha,rho] = x2z(X,C,D(1,:),pathconfig(1));
test = abs(det(rho)) > 1e-12;
if test
    U(:,end+1) = rho\(-alpha-K*Z);
    % Limit Control Values
    UR = 0.5*(U(1,end)+U(2,end));
    UL = 0.5*(U(1,end)-U(2,end));
    if abs(UR) > UMAX
        UR = UMAX*sign(UR);
    end
    if abs(UL) > UMAX
        UL = UMAX*sign(UL);
    end
    U(1,end) = UR+UL;
    U(2,end) = UR-UL;
else
    % If rho Is NOT Invertible, Ask For Max Control Forward
    U(:,end+1) = [2*UMAX; 0];
end

% Begin Simulation
while 1
    
    % Update Time
    T(end+1) = T(end) + dt;
    
    % Integrate State
    X(:,end+1) = rk4(@stateModel,X(:,end),zeros(2,1),U(:,end),C,dt);
    
    % Compute Noisey Measurement
    if pathconfig(1) == 1 && count == 0
        % On A Turn And Distance Measurement Is Called For
        Y(:,end+1) = [NaN; NaN];
        YD(:,end+1) = D(1,:)';
        Yconfig(:,end+1) = NaN;
        
    else
        Y(:,end+1) = measModel(X(:,end),N.*randn(size(N)),C,D(1,:),count);
        YD(:,end+1) = D(1,:)';
        Yconfig(:,end+1) = count;
        
    end
    
    count = count + 1;
    if count > 1
        count = 0;
    end
    
    % Clamp theta, -pi < theta < pi
    while abs(X(3,end)) > pi
        if X(3,end) > pi
            X(3,end) = X(3,end) - 2*pi;
        elseif X(3,end) < -pi
            X(3,end) = X(3,end) + 2*pi;
        end
    end
    
    % Test End Of Path Condition
    if pathtest{1}(X(:,end))
        D = D(2:end,:);
        pathconfig = pathconfig(2:end);
        pathtest = pathtest(2:end);
        if isempty(D)
            break;
        end
    end
    
    % Compute Next Control
    [Z,alpha,rho] = x2z(X(:,end),C,D(1,:),pathconfig(1));
    test = abs(det(rho)) > 1e-12;
    if test
        U(:,end+1) = rho\(-alpha-K*Z);
        % Limit Control Values
        UR = 0.5*(U(1,end)+U(2,end));
        UL = 0.5*(U(1,end)-U(2,end));
        if abs(UR) > UMAX
            UR = UMAX*sign(UR);
        end
        if abs(UL) > UMAX
            UL = UMAX*sign(UL);
        end
        U(1,end) = UR+UL;
        U(2,end) = UR-UL;
    else
        % If rho Is NOT Invertible, Ask For Max Control Forward
        U(:,end+1) = [2*UMAX; 0];
    end
    
end