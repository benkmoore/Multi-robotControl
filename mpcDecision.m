function z = mpcDecision(mpc, x_prev, u_prev)
    X = repmat(x_prev', mpc.predictionHorizon, 1);
    U = repmat(u_prev', mpc.predictionHorizon, 1);
    z0 = setDecisions(mpc,X,U);
    [zLB, zUB] = setBounds(mpc);
    options = optimoptions('fmincon','Algorithm','interior-point','Display','none');
    CostFcn = @(z) mpc.costFct(z(:), mpc);
    [z, cost, ExitFlag] = fmincon(CostFcn, z0, [], [], mpc.A, mpc.B * x_prev, zLB, zUB, [], options);
end