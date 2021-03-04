close all
clear all

% Define state and control dimensions and number of drones
lstN = [5];
lst = [];
for c = lstN
 N = 4;
tic 
    nx = 6 * N;
    nu = 3 * N;

% Set up mpc
mpc.N = N;
mpc.nx = nx;
mpc.nu = nu;
mpc.duration = 30;
mpc.controlHorizon = 5;
mpc.predictionHorizon = 5;
mpc.paddedDuration = mpc.duration + mpc.predictionHorizon;
mpc.dt = 1;
mpc.totalDuration = mpc.duration * mpc.dt;
[mpc.x_d, x_r, y_r] = getDesiredReference(mpc,0,2);
mpc.obstacles = setObstacles();

[mpc.C0, mpc.d0] = getObsPolyhedrons(mpc);

% Define dynamics and cost function
A = [1 mpc.dt;
    0 1];
B = [mpc.dt^2/2;
    mpc.dt];
A_kron = kron(eye(N, N), kron(A, eye(3, 3)));
B_kron = kron(eye(N, N), kron(B, eye(3, 3)));
mpc.stateFct = @(x,u) A_kron * x + B_kron * u;
mpc.costFct = @costFunction;
[z2uMap, u2zMap] = mapControls(mpc);
mpc.z2uMap = z2uMap; mpc.u2zMap = u2zMap;

    % Values to tweak
    mpc.l_d = 2;
    mpc.l_m = 1.75;
    mpc.obs_weight = 0; % cost of obs avoidance in cost fct
    mpc.u_weight = 10^3;
    mpc.x_weight = 10^1;
    mpc.boundX_ub = repmat([Inf,Inf,Inf,10,10,10]', mpc.N, 1);
    mpc.boundU_ub = repmat([2,2,2]', mpc.N, 1);
    mpc.boundX_lb = - mpc.boundX_ub;
    mpc.boundU_lb = - mpc.boundU_ub;

    % Cost on states
    mpc.Q = sparse(kron(eye(N, N), diag([1, 1, 1, 0, 0, 0])) * mpc.x_weight);
    %mpc.Q = A_kron'*A_kron;
    mpc.Qn = sparse(kron(eye(N ,N), diag([10, 10, 10, 10, 10, 10])) * 30);
    % Cost on input
    mpc.P = sparse(kron(eye(3,3), eye(N, N))*mpc.u_weight);

    % Initialize all states
    mpc.x0 = mpc.x_d(:,1);
    mpc.u0 = zeros(mpc.nu, 1);
    mpc.current = 2;

[mpc.A, mpc.B] = setEqConstraints(mpc, A_kron, B_kron);
num_obs = 2;
x_r = mpc.x_d(1:6,mpc.current:mpc.current + mpc.predictionHorizon - 1);
p0 = mpc.x0(1:6);
u0 = mpc.u0(1:3);
% Cost on states
Q = diag([1, 1, 1, 0, 0, 0]) * mpc.x_weight;
% Cost on input
P = eye(3,3)*mpc.u_weight;
A = kron(A, eye(3, 3));
B = kron(B, eye(3, 3));
uBound = [2;2;2];
prep = (reshape(mpc.obstacles(1:2,:)',num_obs,2));

cvx_begin
variables p(6,mpc.predictionHorizon) u(3,mpc.predictionHorizon)

subject to
    x_bar = p - x_r;
    p(:,2:end) == A * p(:,1:end-1) + B * u(:,1:end-1)
    p(:,1) == p0;
    u(:,1) == u0;
    abs(u(:,2:end) - u(:,1:end-1))- repmat(uBound,1,mpc.predictionHorizon-1)  <= 0
    
    p(:,end) == mpc.x_d(1:6,mpc.current + mpc.predictionHorizon - 1);
    for t = 1:mpc.predictionHorizon
        for j = 1:num_obs
            g = []
            cvx_begin
            variables w(2,1)
                minimize (prep(j,:) - p(1:2,t)') * w
                subject to
                    mpc.C0(:,1 + 2*(j-1):2 + 2*(j-1))  * w <= mpc.d0
            cvx_end
            g = [g;w];
        end
        (prep - (p(1:2,t)*ones(1,2))') * p(1:2,t) <= diag((prep - p(1:2,t)) * mpc.obstacles(1:2,:)) + g(t) - (prep - p(1:2,t))
    end
    minimize sum(diag(x_bar'*Q*x_bar)) ...
     + sum(diag(u'*P*u));
cvx_end




%%











x = mpc.x0; u = mpc.u0;
Xdef = [x']; Udef = [u'];
fprintf('iter: %g \n',i)
for t = 1:mpc.duration
    fprintf('%g \n',t)
    [mpc.Ac, mpc.bc] = getFeasibleSpace(mpc, x);
    mpc.g = solveForGs(mpc);
    z = mpcDecision(mpc, x, u);
    [X, U] = getDecisions(z, mpc);
    x = X(1,:)';
    u = U(1,:)';
    Xdef = [Xdef; x'];
    Udef = [Udef; u'];
    mpc.current = mpc.current + 1;

end
%  figure('Color','white');plot(lstN,lst,'-o'),xlabel('Number of drones');ylabel('Runtime (s)'); grid on
%%
% err = error(mpc, Xdef);
h = zeros(1,10);
figure('Color','white');
h(1) = plot3(x_r(:), y_r(:),z_r(:),'-o','MarkerSize',5,'MarkerFaceColor','r','DisplayName','Reference');
hold on;
h(2) = plot3(Xdef(:,1),Xdef(:,2),Xdef(:,3),'-o','MarkerSize',2.5,'MarkerFaceColor','k','DisplayName','Robot 1');
h(3) = plot3(Xdef(:,7),Xdef(:,8),Xdef(:,9),'-o','MarkerSize',2.5,'MarkerFaceColor','b','DisplayName','Robot 2');
h(4) = plot3(Xdef(:,13),Xdef(:,14),Xdef(:,15),'-o','MarkerSize',2.5,'MarkerFaceColor','g','DisplayName','Robot 3');
h(5) = plot3(Xdef(:,19),Xdef(:,20),Xdef(:,21),'-o','MarkerSize',2.5,'MarkerFaceColor','y','DisplayName','Robot 4');
xlabel('x');ylabel('y');zlabel('z'); grid on
title(['Control Horizon =',' ', num2str(mpc.controlHorizon),', Prediction Horizon =',' ', num2str(mpc.predictionHorizon)])

for i = 1 : size(mpc.obstacles,2)
    h = drawcircle(mpc.obstacles(1,i), mpc.obstacles(2,i), mpc.obstacles(4,i), 'r',h);
end
legend(h(1:5))
axis equal
% xlim([-1,10])
% ylim([-1,10])
toc
function h = drawcircle(x,y,r,c,h)
th = 0:pi/50:2*pi;
x_circle = r * cos(th) + x;
y_circle = r * sin(th) + y;
h(end+1) = plot(x_circle, y_circle);
h(end+1) = fill(x_circle, y_circle, c);
end

% errors
function tot_e = error(mpc, Xdef)
    e_tr1 = norm(mpc.x_d(1:2,1:mpc.duration - mpc.predictionHorizon) - Xdef(:,1:2)');
    e_tr2 = norm(mpc.x_d(7:8,1:mpc.duration - mpc.predictionHorizon) - Xdef(:,7:8)');
    e_tr3 = norm(mpc.x_d(13:14,1:mpc.duration - mpc.predictionHorizon) - Xdef(:,13:14)');
    e_tr4 = norm(mpc.x_d(19:20,1:mpc.duration - mpc.predictionHorizon) - Xdef(:,19:20)');
    tot_e = e_tr1 + e_tr2 + e_tr3 + e_tr4;
end

