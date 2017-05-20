function [Wm,Wc] = getWeights(UC,L)
% Compute Weights For UKF
% UC = UKF Constants
%   alpha
%   beta
%   kappa
% W = UKF Weights
%   Wm
%   Wc

alpha = UC(1); beta = UC(2); kappa = UC(3);
lambda = alpha^2*(L+kappa)-L;

W0m = lambda/(L+lambda);
Wim = 1/(2*(L+lambda))*ones(1,2*L);
Wm = [W0m, Wim];

W0c = lambda/(L+lambda)+1-alpha^2+beta;
Wic = Wim;
Wc = [W0c, Wic];