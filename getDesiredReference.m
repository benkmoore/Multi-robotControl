function [x_d, x_r, y_r] = getDesiredReference(mpc, boolPlot, nTraj)
    x_d = zeros(mpc.nx, mpc.paddedDuration);
    switch nTraj
        case 1
            x_dr = [-1/sqrt(2), 1/sqrt(2), 1/sqrt(2), -1/sqrt(2);
                1/sqrt(2), 1/sqrt(2), -1/sqrt(2), -1/sqrt(2);
                0, 0, 0 ,0];
            x_r = 0:8;
            y_r = [0 .15 1.12 2.36 2.36 1.46 .49 .06 0];
            cs = spline(x_r,[0 y_r 0]); 
            x_r = linspace(0, 8, mpc.duration);
            y_r = ppval(cs, x_r);
            y_prime = ppval(fnder(cs,1),x_r);
            prime = [ones(size(y_prime)); y_prime; zeros(size(y_prime))];
            x_d = zeros(mpc.nx, mpc.duration);
            for t = 1:mpc.duration
                if prime(1,t) == 0
                    psi = 0;
                else
                    psi = atan2(prime(1,t),prime(2,t));
                end
                if prime(3,t) == 0
                    gamma = 0;
                else
                    gamma = atan2(prime(3,t),prime(2,t));   
                end
                R = roty(rad2deg(-gamma)) * rotz(rad2deg(-psi));
                x_d(:,t) = reshape([[x_r(t);y_r(t);0] + R * x_dr(:,1:mpc.N); repmat(prime(:,t),1,mpc.N)],mpc.nx,1);
            end
        otherwise
            x_dr = [-1/sqrt(2), 1/sqrt(2), 1/sqrt(2), -1/sqrt(2);
                1/sqrt(2), 1/sqrt(2), -1/sqrt(2), -1/sqrt(2);
                0, 0, 0 ,0];
            x_r = linspace(0, 8, round(0.5 * mpc.duration));
            x_r = [x_r, 8 * ones(1, mpc.duration - round(0.5 * mpc.duration))];
            y_r = zeros(1, round(0.5 * mpc.duration));
            y_r = [y_r, linspace(0, 8, mpc.duration - round(0.5 * mpc.duration))];
            prime = [ones(1, round(0.5 * mpc.duration)), zeros(1, mpc.duration - round(0.5 * mpc.duration));
                zeros(1, round(0.5 * mpc.duration)), ones(1, mpc.duration - round(0.5 * mpc.duration));
                zeros(1, mpc.duration)];
            for t = 1:mpc.duration
                if prime(1,t) == 0
                    psi = 0;
                else
                    psi = atan2(prime(1,t),prime(2,t));
                end
                if prime(3,t) == 0
                    gamma = 0;
                else
                    gamma = atan2(prime(3,t),prime(2,t));   
                end
                R = roty(rad2deg(-gamma)) * rotz(rad2deg(-psi));
                x_d(:,t) = reshape([[x_r(t);y_r(t);0] + R * x_dr(:,1:mpc.N); repmat(prime(:,t),1,mpc.N)],mpc.nx,1);
            end
    end
    x_d(:, mpc.duration + 1: mpc.paddedDuration) = repmat(x_d(:,mpc.duration),1,mpc.predictionHorizon); 
    if (boolPlot)
        figure();
        for t = 1:mpc.duration
            plot(x_r(t), y_r(t),'-or','MarkerSize',5,'MarkerFaceColor','r')
            hold on;
            plot(x_d(1,t),x_d(2,t),'-or','MarkerSize',5,'MarkerFaceColor','k');
            if mpc.N > 1
                plot(x_d(7,t),x_d(8,t),'or','MarkerSize',5,'MarkerFaceColor','b');
            end
            if mpc.N > 2
                plot(x_d(13,t),x_d(14,t),'or','MarkerSize',5,'MarkerFaceColor','g');
            end
            if mpc.N > 3
                plot(x_d(19,t),x_d(20,t),'or','MarkerSize',5,'MarkerFaceColor','y');
            end
            pause(0.4)
        end
    end
end
