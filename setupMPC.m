close all
clear all


N = 4;
nx = 6 * N;
nu = 3 * N;

x_obs1 = 6;
y_obs1 = 0;
z_obs1 = 1;
r_obs1 = 0.1;
x_obs2 = 6;
y_obs2 = 1;
z_obs2 = 1;
r_obs2 = 0.1;
mpc.obstacles = [[x_obs1, y_obs1, z_obs1, r_obs1]',[x_obs2, y_obs2, z_obs2, r_obs2]'];
mpc.l_d = 2;
mpc.l_m = 1.75;
mpc.obs_weight = 6*(10^8); % cost of obs avoidance in cost fct
mpc.u_weight = 10^-17;
mpc.x_weight = 10^12;

mpc.N = N;
mpc.nx = nx;
mpc.nu = nu;
mpc.duration = 50;
mpc.controlHorizon = 2;
mpc.predictionHorizon = 5;
mpc.dt = 0.005;
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
mpc.Q = kron(diag([1, 1, 1, 0, 0, 0]), eye(N, N))*mpc.x_weight;
%mpc.Q = A_kron'*A_kron;
mpc.Qn = kron(diag([10^10, 10^10, 10^10, 10^10, 10^10, 10^10]), eye(N ,N));

% Cost on input
mpc.P = kron(eye(3,3), eye(N, N))*mpc.u_weight;

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

figure();
hold on;
for t = 1:mpc.duration - mpc.predictionHorizon
    plot(mpc.x_d(1,t), mpc.x_d(2,t),'-o','MarkerSize',5,'MarkerFaceColor','r')
    plot(mpc.x_d(7,t), mpc.x_d(8,t),'-o','MarkerSize',5,'MarkerFaceColor','g')
    plot(Xdef(t,1),Xdef(t,2),'-o','MarkerSize',2.5,'MarkerFaceColor','k');
    plot(Xdef(t,7),Xdef(t,8),'-o','MarkerSize',2.5,'MarkerFaceColor','b');
end
for i = 1 : size(mpc.obstacles,2)
    drawcircle(mpc.obstacles(1,i), mpc.obstacles(2,i), mpc.obstacles(4,i), 'r')
end
axis equal

function drawcircle(x,y,r,c)
th = 0:pi/50:2*pi;
x_circle = r * cos(th) + x;
y_circle = r * sin(th) + y;
plot(x_circle, y_circle);
fill(x_circle, y_circle, c)
end

% errors
function tot_e = error(mpc, Xdef)
    e_tr1 = norm(mpc.x_d(1:2,1:mpc.duration - mpc.predictionHorizon) - Xdef(:,1:2)');
    e_tr2 = norm(mpc.x_d(7:8,1:mpc.duration - mpc.predictionHorizon) - Xdef(:,7:8)');
    e_tr3 = norm(mpc.x_d(13:14,1:mpc.duration - mpc.predictionHorizon) - Xdef(:,13:14)');
    e_tr4 = norm(mpc.x_d(19:20,1:mpc.duration - mpc.predictionHorizon) - Xdef(:,19:20)');
    tot_e = e_tr1 + e_tr2 + e_tr3 + e_tr4;
end

