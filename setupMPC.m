N = 4;
nx = 6 * N;
nu = 3 * N;

mpc.N = N;
mpc.nx = nx;
mpc.nu = nu;
mpc.duration = 50;
mpc.controlHorizon = 2;
mpc.predictionHorizon = 5;
mpc.dt = 0.01;
mpc.totalDuration = mpc.duration * mpc.dt;

[mpc.x_d, x_r, y_r] = getDesiredReference(mpc,0);
A = [1 mpc.dt;
    0 1];
B = [mpc.dt^2/2;
    mpc.dt];

A_kron = kron(kron(A, eye(3, 3)), eye(N, N));
B_kron = kron(kron(B, eye(3, 3)), eye(N, N));
mpc.stateFct = @(x,u) A_kron * x + B_kron * u;
mpc.costFct = @costFunction;
mpc.xMin = [-Inf; -Inf; -Inf; -15; -15; -15] * Inf;
mpc.xMax = [Inf; Inf; Inf; 15; 15; 15] * Inf;
mpc.uMin = [-4; -4; -4] * Inf;
mpc.uMax = -mpc.uMin;
mpc.Q = kron(diag([10000, 10000, 10000, 5, 5, 5]), eye(N, N))*100;
mpc.P = kron(eye(3,3), eye(N, N))*0.001;
mpc.Qn = kron(diag([50, 50, 50, 50, 50, 50]), eye(N ,N))*100;
mpc.x0 = mpc.x_d(:,1);%zeros(mpc.nx, 1);
mpc.u0 = zeros(mpc.nu, 1);
mpc.current = 1;

mpc.boundX_ub = repmat([Inf,Inf,Inf,10,10,10]', mpc.N, 1);
mpc.boundU_ub = repmat([5,5,5]', mpc.N, 1);
mpc.boundX_lb = - mpc.boundX_ub;
mpc.boundU_lb = - mpc.boundU_ub;
[z2uMap, u2zMap] = mapControls(mpc);
mpc.z2uMap = z2uMap; mpc.u2zMap = u2zMap;  

[Aeq, Beq] = setEqConstraints(mpc, A_kron, B_kron);
mpc.A = Aeq; mpc.B = Beq;

x = mpc.x0; u = mpc.u0;
Xdef = []; Udef = [];
for t = 1:mpc.duration - mpc.predictionHorizon
    fprintf('%g \n',t)
    z = mpcDecision(mpc, x, u);
    [X, U] = getDecisions(z, mpc);
    x = X(1,:)';
    u = U(1,:)';
    Xdef = [Xdef; x'];
    Udef = [Udef; u'];
    mpc.current = mpc.current + 1;
end



figure();
for t = 1:mpc.duration - mpc.predictionHorizon
    plot(mpc.x_d(1,t), mpc.x_d(2,t),'-or','MarkerSize',5,'MarkerFaceColor','r')
    hold on;
    plot(Xdef(t,1),Xdef(t,2),'-or','MarkerSize',5,'MarkerFaceColor','k');
%     plot(Xdef(t,7),Xdef(t,8),'or','MarkerSize',5,'MarkerFaceColor','b');
%     plot(Xdef(t,13),Xdef(t,14),'or','MarkerSize',5,'MarkerFaceColor','g');
%     plot(Xdef(t,19),Xdef(t,20),'or','MarkerSize',5,'MarkerFaceColor','y');
%     pause(0.4)
end

