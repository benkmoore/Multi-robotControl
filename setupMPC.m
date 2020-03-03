close all
clear all

% weight of u vs x
pows = -20 : -15;
rel_weight = 10.^pows;
errors = [];

for i = 1 : length(rel_weight)
    N = 4;
    nx = 6 * N;
    nu = 3 * N;

    mpc.rho = rel_weight(i);
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

    mpc.x0 = mpc.x_d(:,1);%zeros(mpc.nx, 1);
    mpc.u0 = zeros(mpc.nu, 1);
    mpc.current = 1;

    mpc.boundX_ub = repmat([Inf,Inf,Inf,10,10,10]', mpc.N, 1);
    mpc.boundU_ub = repmat([5,5,5]', mpc.N, 1);
    mpc.boundX_lb = - mpc.boundX_ub;
    mpc.boundU_lb = - mpc.boundU_ub;
    
    % Cost on states
    mpc.Q = kron(diag([1, 1, 1, 0, 0, 0]), eye(N, N));
    %mpc.Q = A_kron'*A_kron;
    mpc.Qn = kron(diag([10^10, 10^10, 10^10, 10^10, 10^10, 10^10]), eye(N ,N));
 
    % Cost on input
    mpc.P = kron(eye(3,3), eye(N, N))*mpc.rho;

    [z2uMap, u2zMap] = mapControls(mpc);
    mpc.z2uMap = z2uMap; mpc.u2zMap = u2zMap;  

    [Aeq, Beq] = setEqConstraints(mpc, A_kron, B_kron);
    mpc.A = Aeq; mpc.B = Beq;

    x = mpc.x0; u = mpc.u0;
    Xdef = []; Udef = [];
    fprintf('iter: %g \n',i)
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
    err = error(mpc, Xdef);
    errors = [errors, err];
    
    figure();
    hold on;
    for t = 1:mpc.duration - mpc.predictionHorizon
        plot(mpc.x_d(1,t), mpc.x_d(2,t),'-o','MarkerSize',5,'MarkerFaceColor','r')
        plot(mpc.x_d(7,t), mpc.x_d(8,t),'-o','MarkerSize',5,'MarkerFaceColor','g')
        plot(Xdef(t,1),Xdef(t,2),'-o','MarkerSize',2.5,'MarkerFaceColor','k');
        plot(Xdef(t,7),Xdef(t,8),'-o','MarkerSize',2.5,'MarkerFaceColor','b');
    end
end

figure()
hold on
plot(rel_weight, errors)
xlabel('rel. weight')
ylabel('Error')
grid on


% errors
function tot_e = error(mpc, Xdef)
    e_tr1 = norm(mpc.x_d(1:2,1:mpc.duration - mpc.predictionHorizon) - Xdef(:,1:2)');
    e_tr2 = norm(mpc.x_d(7:8,1:mpc.duration - mpc.predictionHorizon) - Xdef(:,7:8)');
    e_tr3 = norm(mpc.x_d(13:14,1:mpc.duration - mpc.predictionHorizon) - Xdef(:,13:14)');
    e_tr4 = norm(mpc.x_d(19:20,1:mpc.duration - mpc.predictionHorizon) - Xdef(:,19:20)');
    tot_e = e_tr1 + e_tr2 + e_tr3 + e_tr4;
end

