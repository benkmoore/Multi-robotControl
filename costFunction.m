function J = costFunction(x, u, xd, mpc.Q, mpc.P, mpc.Qn)
% x \in R^(6N x p)
% u \in R^(3N x p)
cost = diag((x(1:end-1)-xd(1:end-1))'*mpc.Q*(x(1:end-1)-xd(1:end-1)) + u'*mpc.P*u);
J = sum(cost(1:end-1)) + ...
    (x(:,p+1) - xd(:,p+1))' * mpc.Qn * (x(:,p+1) - xd(:,p+1));
end