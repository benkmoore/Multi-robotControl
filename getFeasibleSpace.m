function [Ac, bc] = getFeasibleSpace(mpc, x)

    num_obs = size(mpc.obstacles, 2);
    
    % build Ac p and bc for each robot at each time step
    for k = 0 : mpc.predictionHorizon
        for i = 1 : mpc.N
            
            if k == 0
                p0 = x((i-1)*6 + 1 : (i-1)*6 + 2, 1)'; %1 x 2
            else
                p0 = mpc.x_d((i-1)*6 + 1 : (i-1)*6 + 2, mpc.current + k)';
            end

            for j = 1 : num_obs
                q = [mpc.obstacles(1,j), mpc.obstacles(2,j)]; %1 x 2

                Ac(j+k*num_obs,(i-1)*2 + 1: i*2) = q - p0;

                bc(j+k*num_obs,i) = (q - p0)*q';

            end

        end
    end

end

