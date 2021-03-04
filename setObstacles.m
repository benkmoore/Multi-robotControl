function obstacles = setObstacles()
    x_obs1 = 4;
    y_obs1 = 0;
    z_obs1 = 1;
    r_obs1 = 0.2;
    x_obs2 = 7;
    y_obs2 = 1;
    z_obs2 = 1;
    r_obs2 = 0.2;
    obstacles = [[x_obs1, y_obs1, z_obs1, r_obs1]',[x_obs2, y_obs2, z_obs2, r_obs2]'];
end
