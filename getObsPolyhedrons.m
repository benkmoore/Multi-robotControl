function [C, d] = getObsPolyhedrons(mpc)
    
    num_obs = size(mpc.obstacles, 2);
    mpc.numObsfaces = 4;
    
    C = zeros(mpc.numObsfaces, 2*num_obs);
    d = zeros(mpc.numObsfaces, num_obs);
    
    for i = 1 : num_obs
        q0 = [mpc.obstacles(1,i), mpc.obstacles(2,i)]; %1 x 2
        q_up = [q0(1), q0(2)+mpc.obstacles(4,i)];
        q_low = [q0(1), q0(2)-mpc.obstacles(4,i)];
        q_r = [q0(1)+mpc.obstacles(4,i), q0(2)];
        q_l = [q0(1)-mpc.obstacles(4,i), q0(2)];
        
        C(:,(i-1)*2 + 1: i*2) = [ q_up - q0 ;  % each C0 4 x 2
                                  q_low - q0;
                                  q_r - q0;
                                  q_l - q0 ];
                              
        d(:,i) = [ (q_up - q0)*q_up' ;  % each d0 4 x 1
                  (q_low - q0)*q_low';
                  (q_r - q0)*q_r';
                  (q_l - q0)*q_l' ];
        
    end

end

