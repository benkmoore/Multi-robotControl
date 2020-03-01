N = 4;
nx = 6 * N;
nu = 3 * N;

mpc.N = N;
mpc.nx = nx;
mpc.nu = nu;
mpc.duration = 100;
mpc.controlHorizon = 2;
mpc.predictionHorizon = 5;
mpc.dt = 0.1;
mpc.totalDuration = mpc.duration * mpc.dt;

mpc.x_d = getDesiredReference(mpc,0);
A = [1 mpc.dt;
    0 1];
B = [mpc.dt^2/2;
    mpc.dt];

A_kron = kron(kron(A, eye(3, 3)), eye(N, N));
B_kron = kron(kron(B, eye(3, 3)), eye(N, N));
mpc.stateFct = @(x,u) A_kron * x + B_kron * u;
mpc.costFct = @costFunction;
mpc.xMin = [-Inf; -Inf; -Inf; -15; -15; -15];
mpc.xMax = [Inf; Inf; Inf; 15; 15; 15];
mpc.uMin = [-4; -4; 4];
mpc.uMax = -mpc.uMin;
mpc.Q = kron(diag([10, 10, 10, 5, 5, 5]), eye(N, N));
mpc.P = kron(eye(3,3), eye(N, N));
mpc.Qn = kron(diag([50, 50, 50, 50, 50, 50]), eye(N ,N));
mpc.x0 = zeros(mpc.nx, 1);
mpc.u0 = zeros(mpc.nu, 1);
mpc.current = 1;

mpc.boundX_ub = repmat([Inf,Inf,Inf,10,10,10]', mpc.N, 1);
mpc.boundU_ub = repmat([5,5,5]', mpc.N, 1);
mpc.boundX_lb = - mpc.boundX_ub;
mpc.boundU_lb = - mpc.boundU_ub;

[Aeq, Beq] = setEqConstraints(mpc, A_kron, B_kron);
mpc.A = Aeq; mpc.B = Beq;

z0 = mpcDecision(mpc, mpc.x0, mpc.u0);








