function g = solveForGs(mpc)

    num_obs = size(mpc.obstacles, 2);
    for i = 1 : mpc.N % for each robot
        for k = 1 : mpc.predictionHorizon + 1 % loop over future times and current
            Ac = mpc.Ac((k-1)*num_obs+1:k*num_obs,(i-1)*2 + 1: i*2);
            for j = 1 : num_obs
                cvx_begin quiet
                    variable w(2)
                    minimize Ac(j,:)*w
                    subject to
                        mpc.C0(:,(j-1)*2 + 1: j*2)*w <= mpc.d0(:,j);
                cvx_end
                g(i,k,j) = cvx_optval; %each robot, each time, each obs
            end
        end
    end

end

