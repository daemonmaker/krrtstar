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
system = 'snglint2dfullyobservable'

% Set dimensionality information for system
if strcmp(system, 'snglint2dfullyobservable')
    % Single integrator
    X_DIM = 2
    U_DIM = 2
    Z_DIM = 2
elseif strcmp(system, 'snglint2dpartiallyobservable')
    % Single integrator
    X_DIM = 2
    U_DIM = 1
    Z_DIM = 1
elseif strcmp(system, 'dblint1dfullyobservable')
    % Double integrator 1D
    X_DIM = 2
    U_DIM = 1
    Z_DIM = 1
elseif strcmp(system, 'dblint2dfullyobservable')
    % Double integrator 2D
    X_DIM = 4
    U_DIM = 2
    Z_DIM = 4
elseif strcmp(system, 'dblint2dpartiallyobservable')
    % Double integrator 2D
    X_DIM = 4
    U_DIM = 2
    Z_DIM = 2
elseif strcmp(system, 'dblint2din3dfullyobservable')
    % Double integrator 2D - embedded in 3D
    X_DIM = 5
    U_DIM = 2
    Z_DIM = 5
end

% Setup space for system
A = zeros(X_DIM, X_DIM);
B = zeros(X_DIM, U_DIM);
C = zeros(Z_DIM, X_DIM);
D = zeros(Z_DIM, U_DIM);
M = eye(X_DIM, X_DIM);
N = eye(Z_DIM, Z_DIM);
R = zeros(U_DIM, U_DIM);

%% Setup system
if strcmp(system, 'snglint2dfullyobservable')
    B(1, 1) = 1;
    B(2, 2) = 1
    C(1, 1) = 1;
    C(2, 2) = 1
    R = eye(U_DIM)*0.25;
    %R = ones(U_DIM, U_DIM);
    %R(1, 2) = 0.25;
    %R(2, 1) = 0.25
    
    % Enlarged uniformly - USED FOR EXPERIMENTS SNGLINT2DFOv1
    m_multiplier = 1
    M(1,1) = 0.25;
    M(2,2) = 0.5;
    n_multiplier = 0.1
    
    % Skewed - USED FOR EXPERIMENTS SNGLINT2DFOv2
%     m_multiplier = 0.01
%     M(1,1) = 0.1;
%     M(2,2) = 1;
%     n_multiplier = 1
%     N = N*0.5;
%     N(1,1) = 0.25;


elseif strcmp(system, 'snglint2dpartiallyobservable')
    % INCOMPLETE
   
    B(1, 1) = 1
    C(1, 1) = 1
    R = eye(U_DIM)*0.25;
    %R = ones(U_DIM, U_DIM);
    %R(1, 2) = 0.25;
    %R(2, 1) = 0.25

    m_multiplier = 1
    
    N = N*0.5

elseif strcmp(system, 'dblint1dfullyobservable')
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
    C = eye(Z_DIM)

% Used in
% Visualization of vandenberg map
if false
    % X noise - skewed
    m_multiplier = 1;
    M(1,1) = 1;
    M(2,2) = 0.1;
    M(3,3) = 1;
    M(4,4) = 0.1;
    n_multiplier = 0.1
elseif false
    % Y noise - skewed
    m_multiplier = 1;
    M(1,1) = 0.1;
    M(2,2) = 1;
    M(3,3) = 0.1;
    M(4,4) = 1;
    n_multiplier = 0.1
elseif false
    % Y noise - skewed
    m_multiplier = 2;
    M(1,1) = 0.25;
    M(1,2) = 0.5;
    M(2,2) = 0.25;
    M(3,3) = 0.1;
    M(4,4) = 0.1;
    n_multiplier = 0.5
%     N(1,1) = 0.1;
%     N(3,3) = 0.1;
end

% Used in
% Simulations for DBLINT2D_FO
if false
    % X noise - skewed
    m_multiplier = 1;
    M(1,1) = 0.5;
    M(2,2) = 0.1;
    M(3,3) = 0.5;
    M(4,4) = 0.1;
    n_multiplier = 0.1
elseif false
    % Y noise - skewed
    m_multiplier = 1;
    M(1,1) = 0.1;
    M(2,2) = 0.5;
    M(3,3) = 0.1;
    M(4,4) = 0.5;
    n_multiplier = 0.1
elseif true
    % Y noise - skewed
    m_multiplier = 1;
    M(1,1) = 0.25;
    M(1,2) = 0.5;
    M(2,2) = 0.25;
    M(3,3) = 0.1;
    M(4,4) = 0.1;
    n_multiplier = 0.5
%     N(1,1) = 0.1;
%     N(3,3) = 0.1;
end

elseif strcmp(system, 'dblint2dpartiallyobservable')
    A(1, 3) = 1;
    A(2, 4) = 1
    B(3, 1) = 1;
    B(4, 2) = 1
    C(1, 1) = 1;
    C(2, 2) = 1
 
    % Enlarged uniformly - USED FOR EXPERIMENTS DBLINT2DPOv1
%     m_multiplier = 1;
%     M(1,1) = 1;
%     M(2,2) = 1;
%     M(3,3) = 1;
%     M(4,4) = 1;
%     n_multiplier = 0.01;
%     N(1,1) = 1;
%     N(2,2) = 1;

    % Rotated & skewed - USED FOR EXPERIMENTS DBLINT2DFOv2
    m_multiplier = 100;
    M(1,1) = 0.1;
    M(2,2) = 0.1;
    M(3,3) = 0.00001;
    M(4,4) = 0.00001;
    n_multiplier = 0.01;
    N(1,1) = 1;
    N(2,2) = 1;

%     M(1,1) = 0.01;
%     M(1,2) = 0.001;
%     M(1,3) = 0.00001;
%     M(3,1) = 0.00001;
%     M(1,4) = 0.000001;
%     M(4,1) = 0.000001;
%     M(2,1) = 0.001;
%     M(2,2) = 0.01;
%     M(2,3) = 0.0001;
%     M(3,2) = 0.0001;
%     M(2,4) = 0.00001;
%     M(4,2) = 0.00001;
%     M(3,3) = 0.001;
%     M(3,4) = 0.0001;
%     M(4,3) = 0.0001;
%     M(4,4) = 0.001;
    %[M,p] = chol(M*transpose(M)) % p = (positive value) indicates M is NOT positive semidefinite

    
    %N = N*0.1;

%     N(1,1) = 0.00005;
%     N(1,2) = 0.0000025;
%     N(2,1) = 0.0000025;
%     N(2,2) = 0.00005
elseif strcmp(system, 'dblint2din3dfullyobservable')
    A(1, 3) = 1;
    A(2, 4) = 1;
    A(3, 5) = 1
    B(4, 1) = 1;
    B(5, 2) = 1
    C = eye(Z_DIM)

    % X noise - skewed
    % Uwsed in
    % Visualization of vandenberg map
    m_multiplier = 1;
    M(1,1) = 1;
    M(2,2) = 0.1;
    M(3,3) = 1;
    M(4,4) = 0.1;
    n_multiplier = 0.001
    
    % Y noise - skewed
    % Uwsed in
    % Visualization of vandenberg map
    m_multiplier = 1;
    M(1,1) = 0.1;
    M(2,2) = 1;
    M(3,3) = 0.1;
    M(4,4) = 1;
    n_multiplier = 0.001
%     N(1,1) = 0.1;
%     N(3,3) = 0.1;

end

% Only needed if M & N are covariance matrices
M = M*m_multiplier
%M_orig = M
%M = chol(M*transpose(M))
% 
N = N*n_multiplier
% N_orig = N
% N = chol(N*transpose(N))

sys = ss(A, B, C, D)
minreal(sys)

%% Solve CARE
[P, L, G, report] = care(transpose(A), transpose(C)*inv(transpose(N)), M*transpose(M))

%% Calculate K
K = P*transpose(C)*inv(N*transpose(N))

%% Solve LQR problem
% L can be calculated using (infinite horizon) LQR function matlab
% Can start with the penalty matrices Q = I, R = I to start (these are not
% the same Q as used in the paper draft.

Q_penalty = eye(X_DIM, X_DIM)
N_penalty = zeros(X_DIM, U_DIM)
R_penalty = eye(U_DIM, U_DIM)
[L, S, e] = lqr(A, B, Q_penalty, R_penalty, N_penalty);
L

%% Setup combined system
BL = B*L
KC = K*C
KN = K*N
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
%final = chol(x_sigma, 'lower')
final = chol(x_sigma(1:2, 1:2), 'lower')
%final = final(1:2, 1:2)
%final = x_sigma(1:2, 1:2)

[U, S, V] = svd(final);

% R = transpose(V)*transpose(U)
% d = sign(det(R))
% if (d == -1)
%     a = R(:, 1);
%     R(:, 1) = R(:, 2);
%     R(:, 2) = a;
% end
% R

x_sigma(1:2, 1:2)
S
V
det(V)
U
det(U)

V = U

det_of_rot = int8(det(V))

if (det_of_rot == -1)
    a = S(1, 1);
    S(1, 1) = S(2, 2);
    S(2, 2) = a;
    
    a = V(:, 1);
    V(:, 1) = V(:, 2);
    V(:, 2) = a;

%     a = V(1, :);
%     V(1, :) = V(2, :);
%     V(2, :) = a;

    a = U(:, 1);
    U(:, 1) = U(:, 2);
    U(:, 2) = a;
end
S
V
transpose(U)
U

% S = S(1:2, 1:2)
% V = V(1:2, 1:2)

%S = S(1:2, 1:2)
%V = V(1:3, 1:3)

det(V)

%% Save the parameters
file_path = '../krrtstar/krrtstar/parameters.txt'
save(file_path, 'A', 'B', 'C', 'M', 'N', 'K', 'L', 'R_tilde', 'S', 'V', 'Sigma', 'x_sigma', 'u_sigma', '-mat')

S
V