close all
clear all

% Define state and control dimensions and number of drones
N = 4;
nx = 6 * N;
nu = 3 * N;

% Set up mpc
mpc.N = N;
mpc.nx = nx;
mpc.nu = nu;
mpc.duration = 50;
mpc.controlHorizon = 2;
mpc.predictionHorizon = 5;
mpc.paddedDuration = mpc.duration + mpc.predictionHorizon;
mpc.dt = 1;
mpc.totalDuration = mpc.duration * mpc.dt;
[mpc.x_d, x_r, y_r] = getDesiredReference(mpc,0,2);
mpc.obstacles = setObstacles();
mpc.threshold = 0.1; % distance to keep from obstacles

% Define dynamics and cost function
A = [1 mpc.dt;
    0 1];
B = [mpc.dt^2/2;
    mpc.dt];
A_kron = kron(eye(N, N), kron(A, eye(3, 3)));
B_kron = kron(eye(N, N), kron(B, eye(3, 3)));
mpc.stateFct = @(x,u) A_kron * x + B_kron * u;
mpc.costFct = @costFunction;
mpc.nonlCon = @nonlcon;
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
mpc.Qn = sparse(kron(eye(N ,N), diag([10, 10, 10, 10, 10, 10])) * 10);
% Cost on input
mpc.P = sparse(kron(eye(3,3), eye(N, N))*mpc.u_weight);

% Initialize all states
mpc.x0 = mpc.x_d(:,1);
mpc.u0 = zeros(mpc.nu, 1);
mpc.current = 2;

[mpc.A, mpc.B] = setEqConstraints(mpc, A_kron, B_kron);

x = mpc.x0; u = mpc.u0;
Xdef = [x']; Udef = [u'];
fprintf('iter: %g \n',i)
for t = 1:mpc.duration
    fprintf('%g \n',t)
    z = mpcDecision(mpc, x, u);
    [X, U] = getDecisions(z, mpc);
    x = X(1,:)';
    u = U(1,:)';
    Xdef = [Xdef; x'];
    Udef = [Udef; u'];
    mpc.current = mpc.current + 1;
end
% err = error(mpc, Xdef);

figure();
hold on;
plot(x_r(:), y_r(:),'-o','MarkerSize',5,'MarkerFaceColor','r')    
plot(Xdef(:,1),Xdef(:,2),'-o','MarkerSize',2.5,'MarkerFaceColor','k');
plot(Xdef(:,7),Xdef(:,8),'-o','MarkerSize',2.5,'MarkerFaceColor','b');
plot(Xdef(:,13),Xdef(:,14),'-o','MarkerSize',2.5,'MarkerFaceColor','g');
plot(Xdef(:,19),Xdef(:,20),'-o','MarkerSize',2.5,'MarkerFaceColor','y');
for i = 1 : size(mpc.obstacles,2)
    drawcircle(mpc.obstacles(1,i), mpc.obstacles(2,i), mpc.obstacles(4,i), 'r')
end
axis equal
xlim([-1,10])
ylim([-1,10])

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

