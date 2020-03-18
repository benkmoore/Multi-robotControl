function J = costFunction(z, mpc)
[X, U] = getDecisions(z, mpc);
X = X'; U = U';

% Normalize state and input
x_bar = (X(:,1:end-1)-mpc.x_d(:,mpc.current:mpc.current + mpc.predictionHorizon - 2));
% norm_x_bar = norm(x_bar);
% if norm_x_bar > 0
%     x_bar = x_bar/norm_x_bar;
% end
% 
u_bar = U;
% norm_U = norm(U);
% if norm_U > 0
%     u_bar = u_bar/norm_U;
% end

[l_s, l_sp, collision_bin] = getDistObs(X, mpc);
D = l_s.*repmat(collision_bin,1,mpc.predictionHorizon);
D(D == 0) = 1;
D = D.^-1;
D(D == 1) = 0;

cost = sum(diag(x_bar'*mpc.Q*x_bar)) ... % ...  % traj deviation cost
      + sum(diag(u_bar'*mpc.P*u_bar)) ... %input cost
      + (0)*sum(sum(-log(l_s))) + ...
      + 0*sum(sum(D,2));%obstacle avoidance cost
J = cost + ...
    ((X(:,end) - mpc.x_d(:,mpc.current + mpc.predictionHorizon - 1)))' * mpc.Qn * ...
        ((X(:,end) - mpc.x_d(:,mpc.current + mpc.predictionHorizon - 1)));
end