function [zUB, zLB] = getBounds(mpc)
% z = [x; u]
zLB = repmat(mpc.xMin, 6*mpc.N, mpc.p);
zUB = repmat(mpc.xMax, 6*mpc.N, mpc.p);
zLB = [zLB; repmat(mpc.uMin, 3*mpc.N, mpc.p)];
zUB = [zUB; repmat(mpc.uMax, 3*mpc.N, mpc.p)];
end
