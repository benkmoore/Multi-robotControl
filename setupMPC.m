N = 4;
nx = 6 * N;
nu = 3 * N;

mpc.N = N;
mpc.nx = nx;
mpc.nu = nu;
mpc.duration = 40;
mpc.controlHorizon = 4;
mpc.predictionHorizon = 30;
mpc.dt = 0.3;

x_dr = [-1/sqrt(2), 1/sqrt(2), 1/sqrt(2), -1/sqrt(2);
    1/sqrt(2), 1/sqrt(2), -1/sqrt(2), -1/sqrt(2);
    0, 0, 0 ,0];

x_r = 0:8;
y_r = [0 .15 1.12 2.36 2.36 1.46 .49 .06 0];
cs = spline(x_r,[0 y_r 0]); 
x_r = linspace(0, 8, mpc.duration);
y_r = ppval(cs, x_r);
y_prime = ppval(fnder(cs,1),x_r);
prime = [ones(size(y_prime)); y_prime; zeros(size(y_prime))];
x_d = zeros(mpc.nx, mpc.duration); lst = []
for t = 1:mpc.duration
    if prime(1,t) == 0
        psi = 0;
    else
        psi = atan2(prime(1,t),prime(2,t));
    end
    if prime(3,t) == 0
        gamma = 0;
    else
        gamma = atan2(prime(3,t),prime(2,t));   
    end
    lst = [lst, gamma];
    R = roty(rad2deg(-gamma)) * rotz(rad2deg(-psi));
    x_d(:,t) = reshape([[x_r(t);y_r(t);0] + R * x_dr; repmat(prime(:,t),1,N)],6*N,1);
end
figure();
for t = 1:mpc.duration
plot(x_r(t), y_r(t),'-or','MarkerSize',5,'MarkerFaceColor','r')
hold on;
plot(x_d(1,t),x_d(2,t),'-or','MarkerSize',5,'MarkerFaceColor','k');
plot(x_d(7,t),x_d(8,t),'or','MarkerSize',5,'MarkerFaceColor','b');
plot(x_d(13,t),x_d(14,t),'or','MarkerSize',5,'MarkerFaceColor','g');
plot(x_d(19,t),x_d(20,t),'or','MarkerSize',5,'MarkerFaceColor','y');
pause(0.4)

end
figure()
plot(lst)
mpc.nx = nx;
mpc.nu = nu;
mpc.duration = 40;
mpc.controlHorizon = 4;
mpc.predictionHorizon = 30;
mpc.dt = 0.3;
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
mpc.x0 = zeros(6 * mpc.N, 1);
mpc.u0 = zeros(3 * mpc.N, 1);

