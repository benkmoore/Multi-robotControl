function [X, U] = getDecisions(z, mpc)

X = reshape(z(1:mpc.predictionHorizon * mpc.nx), mpc.nx, mpc.predictionHorizon)';
u_z = z(mpc.predictionHorizon*mpc.nx+1:end);
U = reshape(mpc.z2uMap*u_z, mpc.nu,mpc.predictionHorizon)';

end