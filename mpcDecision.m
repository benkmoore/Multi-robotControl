function z = mpcDecision(mpc, x, u_star_prev)
    [z2uMap, u2zMap] = mapControls(mpc);
    mpc.z2uMap = z2uMap; mpc.u2zMap = u2zMap; 
    X0 = repmat(x', mpc.predictionHorizon, 1);
    U0 = repmat(u_star_prev',mpc.predictionHorizon, 1);
    z0 = setDecisions(mpc,X0,U0);
    [zLB, zUB] = setBounds(mpc);
    options = optimoptions('fmincon','Algorithm','sqp','Display','iter');
    CostFcn = @(z) mpc.costFct(z(:), mpc);
    [z, cost, ExitFlag] = fmincon(CostFcn, z0, [], [], mpc.A, mpc.B * x, zLB, zUB, [], options);
end