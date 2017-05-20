function K = computeControlGain(A,B,Q,R)
% Linear Quadratic Regulator Design:
% Compute optimal control gain, K, that minimizes the quantity,
%   J = Integral {x'Qx + u'Ru} dt
% for U = -Kx
% K is given by, K = inv(R)*B'*P
% Where P is given by the solution to the A.R.E.,
% 0 = A'*P + P*A - P*(B*inv(R)*B')*P + Q

epsilon = 1e-12;            % MATLAB Zero
H = [A, -B/R*B'; -Q, -A'];  % Hamiltonian Matrix
[V,D] = eig(H);
DD = diag(D)';

% Check That H Is In Domain Ric
% (i) No Eigenvalues On The Imaginary Axis
test = all(abs(DD) > epsilon);
if test
    
    W = V(:,real(DD) < 0);
    r = size(W,1);
    X1 = W(1:r/2,:);
    X2 = W(r/2+1:end,:);
    
    % (ii) X1 Is Invertible
    test = abs(det(X1)) > epsilon;
    if test
        P = real(X2/X1);
    else
        fprintf('H NOT In Domain Ric\n');
    end
else
    fprintf('H NOT In Domain Ric\n');
end
K = R\B'*P;