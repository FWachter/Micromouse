function XS = generateSigma(X,P,UC)
% Generate Sigma Points For UKF
% X = Augmented State Vector
%   x
%   y
%   theta
%   vt
%   wt
%   v1
%   v2
%   n1
%   n2
%   n3
%   n4
% P = Augmented Covariance Matrix
% UC = UKF Constants
%   alpha
%   beta
%   kappa

alpha = UC(1); kappa = UC(3);
L = length(X);
lambda = alpha^2*(L+kappa)-L;
A = sqrt(L+lambda)*chol(P)';
XS = [X, X*ones(1,L)+A, X*ones(1,L)-A];