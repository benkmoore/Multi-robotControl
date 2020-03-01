function z = setDecisions(mpc, X, U)
    u_z = mpc.u2zMap * reshape(U', mpc.predictionHorizon * mpc.nu,1);
    x_z = reshape(X', mpc.predictionHorizon*mpc.nx,1);
    z = [x_z;u_z];
end