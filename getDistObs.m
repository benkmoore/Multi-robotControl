function [l_s, l_sp, collision_bin]  = getDistObs(X, mpc)
    % X = (6*N) * predictHorizon
    % 2D
    p_robots_x = X(1:6:end,:);
    p_robots_y = X(2:6:end,:);
    
    v_robots_x = X(4:6:end,:);
    v_robots_y = X(5:6:end,:);
    w = atan2(v_robots_y,v_robots_x) + 2*pi; % add 2pi to avoid diving by zero
    
    num_obs = size(mpc.obstacles,2);
    obs_x = mpc.obstacles(1,:);
    obs_y = mpc.obstacles(2,:);
    obs_r = mpc.obstacles(4,:);
    
    l_s = zeros(mpc.N*num_obs,mpc.predictionHorizon);
    l_sp = zeros(mpc.N*num_obs,mpc.predictionHorizon);
    collision_bin = zeros(mpc.N*num_obs,1);
    for i = 1 : num_obs
        dx = p_robots_x - obs_x(i);
        dy = p_robots_y - obs_y(i);
        
        l_s(mpc.N*(i-1)+1:mpc.N*i,1:mpc.predictionHorizon) = abs((( dx.^2 + dy.^2 ).^0.5) - obs_r(i));
        l_sp(mpc.N*(i-1)+1:mpc.N*i,1:mpc.predictionHorizon) = ...
            abs(cot(w).*(-dx) - dy) ./ (cot(w).^2 + 1).^0.5;
        
        % Apply danger and margin radia
        l_s_curr_t = l_s(mpc.N*(i-1)+1:mpc.N*i,1);
        l_sp_curr_t = l_sp(mpc.N*(i-1)+1:mpc.N*i,1);
        
        collision_bin(mpc.N*(i-1)+1:mpc.N*i,1) = (abs(l_s_curr_t) < mpc.l_d).*(abs(l_sp_curr_t) < mpc.l_m);
    end
    

end

