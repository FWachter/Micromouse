function [Xhat,P,T] = runEstimator(Xhat0,PX,PV,PN,Y,YD,Yconfig,TY,U,C)

% Length Of The Augmented State Vector
L = size(PX,2)+size(PV,2)+size(PN,2);

% UT Constants
alpha = 1e-3;
beta = 2;
kappa = 0;
UC = [alpha, beta, kappa];

% Weights For UT Sigma Points
[Wm,Wc] = getWeights(UC,L);

% Allocate Memory For State Estimate And Covariance
T = [0, TY];
lT = length(T);
Xhat = zeros(5,lT);
P = zeros(5,5,lT);
Xhat(:,1) = Xhat0;
P(:,:,1) = PX;


for ii = 1:lT-1
    
    % Time Update
    
    % Generate Sigma Points
    XA = [Xhat(:,ii); zeros(6,1)];
    PA = [P(:,:,ii), zeros(5,6); zeros(2,5), PV, zeros(2,4); zeros(4,7), PN];
    XS = generateSigma(XA,PA,UC);
    XXS = XS(1:5,:);
    XVS = XS(6:7,:);
    XNS = XS(8:end,:);
    
    % Propogate State Forward To Measurement Time
    dt = T(ii+1)-T(ii);
    XU = rk4(@stateModel,XXS,XVS,U(:,ii),C,dt);
    Xhatp = XU*Wm';
    Pp = (XU-Xhatp*ones(1,2*L+1))*diag(Wc)*(XU-Xhatp*ones(1,2*L+1))';
    
    % Measurement Update
    if isnan(Yconfig(ii)) % NOTE: For simulation
        % No Measurement Update
        Xhat(:,ii+1) = Xhatp;
        P(:,:,ii+1) = Pp;
        
    else
        % Valid Measurement
        YS = measModel(XXS,XNS,C,YD(:,ii),Yconfig(ii));
        Yhatp = YS*Wm';
        
        Pyy = (YS-Yhatp*ones(1,2*L+1))*diag(Wc)*(YS-Yhatp*ones(1,2*L+1))';
        Pxy = (XU-Xhatp*ones(1,2*L+1))*diag(Wc)*(YS-Yhatp*ones(1,2*L+1))';
        K = Pxy/Pyy;
        
        Xhat(:,ii+1) = Xhatp + K*(Y(:,ii)-Yhatp);
        P(:,:,ii+1) = Pp - K*Pyy*K';
        
    end
    
    
end
    


