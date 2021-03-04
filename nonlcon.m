function [c, ceq] = nonlcon(z, mpc)
    [X, ~] = getDecisions(z, mpc);
    p_robots_x = X(:,1:6:end);
    p_robots_y = X(:,2:6:end);
    c = [];
    ceq = [];
    for i = 1 : size(mpc.obstacles,2)
        dx = p_robots_x - mpc.obstacles(1,i);
        dy = p_robots_y - mpc.obstacles(2,i); 
        c  = [c;reshape(-(( dx.^2 + dy.^2 ).^0.5) + mpc.obstacles(4,i) + mpc.threshold,mpc.predictionHorizon * mpc.N,1)];
    end    
end
