% Solving for P in AP + PA^T + MM^T - PC^T(NN^T)^-1CP = 0 where matlab uses
% the syntax A^TX + XA - XBB^TX + Q = 0
% A = dynamics matrix
% C = observation matrix
% N = observation noise matrix
% B*transpose(B) = transpose(C)*inv(N*transpose(N))*C
%       => B = transpose(C)*inv(tranpose(N))
% M = motion model noise matrix
% Q = M*transpose(M)

%% Select system and setup basic variables
clear
system = 'snglint2d'

% Set dimensionality information for system
if strcmp(system, 'snglint2d')
    % Single integrator
    X_DIM = 2
    U_DIM = 2
    Z_DIM = 2
elseif strcmp(system, 'dblint1d')
    % Double integrator 1D
    X_DIM = 2
    U_DIM = 1
    Z_DIM = 1
elseif strcmp(system, 'dblint2dfullyobservable')
    % Double integrator 2D
    X_DIM = 4
    U_DIM = 2
    Z_DIM = 4
end

% Setup space for system
A = zeros(X_DIM, X_DIM);
B = zeros(X_DIM, U_DIM);
C = zeros(Z_DIM, X_DIM);
M = eye(X_DIM, X_DIM);
N = eye(Z_DIM, Z_DIM);
R = zeros(U_DIM, U_DIM);

%% Setup system
if strcmp(system, 'snglint2d')
    B(1, 1) = 1;
    B(2, 2) = 1
    C(1, 1) = 1;
    C(2, 2) = 1
    R = ones(U_DIM, U_DIM);
    R(1, 2) = 0.25;
    R(2, 1) = 0.25
    
    %M(1, 1) = 10;
    %M(1, 1) = 0;
    %M(2, 2) = 0;
    %N(1, 1) = 0;
    %N(2, 2) = 0;
    M(1,1) = M(1,1)*0.01;
    M(2,2) = M(2,2)*1
    N = N*0.1
elseif strcmp(system, 'dblint1d')
    A(1, 2) = 1
    B(2, 1) = 1
    C(1, 1) = 1
    C(2, 2) = 1
    R = ones(U_DIM, U_DIM)
elseif strcmp(system, 'dblint2dfullyobservable')
    A(1, 3) = 1;
    A(2, 4) = 1
    B(3, 1) = 1;
    B(4, 2) = 1
    C = eye(4)
    %M = M*0.0000001
    %N = N*0.0000001
    M = M*0
    N = N*0
end

%% Solve CARE
[P, L, G] = care(A, transpose(C)*inv(transpose(N)), M*transpose(M));
P

%% Calculate K
K = P*transpose(C)*inv(N*transpose(N))

%% Solve LQR problem
% L can be calculated using (infinite horizon) LQR function matlab
% Can start with the penalty matrices Q = I, R = I to start (these are not
% the same Q as used in the paper draft.

Q_penalty = eye(X_DIM, X_DIM)*0.1;
N_penalty = zeros(X_DIM, U_DIM)*0.1;
R_penalty = eye(U_DIM, U_DIM)*0.1;
[L, S, e] = lqr(A, B, Q_penalty, R_penalty, N_penalty);
L

%% Setup combined system
BL = B*L;
KC = K*C;
KN = K*N;
F = [A -BL; KC (A - BL - KC)]
G = [M zeros(X_DIM, size(KN, 2)); zeros(size(KN, 1), X_DIM) KN]

%% Solve Lyapunov
% Solving for \Sigma in F\Sigma + \SigmaF^T + GG^T where matlab uses the
% syntax AX + XA^T + Q
% A = F = combined natural dynamics as a function of L and K
% G = combined noise models as a function of K

Q = G*transpose(G)
Sigma = lyap(F, Q)

%% Calculate the state and control covariance matrices
X = [eye(X_DIM, X_DIM) zeros(X_DIM, X_DIM)]
U = [zeros(U_DIM, X_DIM) -L]
x_sigma = X*Sigma*transpose(X)
u_sigma = U*Sigma*transpose(U)

%% Calculate the modified kRRT* penalty matrix
k = 1 + trace(R*u_sigma)

R_tilde = R/k

%% Calculate final covariance matrix and decompose into rotation and scale
% matrices. It's important that the determinant of the rotation matrix be
% 1. If this is not the case then the order of any two columns can be
% swapped to make it the case. However doing so requires that the
% associated rows of the scale matrix be swapped too.
final = chol(x_sigma, 'lower')

[U, S, V] = svd(final)
det(U)
det(V)

%%
