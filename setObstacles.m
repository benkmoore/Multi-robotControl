function obstacles = setObstacles(c)
    x_obs1 = 4;
    y_obs1 = 0;
    z_obs1 = 0;
    r_obs1 = 0.3;
    x_obs2 = 7;
    y_obs2 = 1;
    z_obs2 = 0;
    r_obs2 = 0.3;
    
    obstacles = [[x_obs1, y_obs1, z_obs1, r_obs1]',[x_obs2, y_obs2, z_obs2, r_obs2]'];
    obstacles = [2 + (10-2) * rand(1,c);
                4 * rand(1,c);
                 zeros(1,c);
                 0.5 * rand(1,c)];
end
