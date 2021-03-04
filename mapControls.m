function [z2uMap, u2zMap] = mapControls(mpc)
    totalLength = mpc.controlHorizon * mpc.nu;
    m = [ones(1,mpc.controlHorizon-1), ...
        mpc.predictionHorizon - mpc.controlHorizon + 1];
    z2uMap = zeros(mpc.predictionHorizon * mpc.nu, totalLength);
    u2zMap = z2uMap';
    ix = 1:mpc.nu;
    jx = 1:mpc.nu;
    for i = 1:length(m)
        u2zMap(ix,jx) = eye(mpc.nu, mpc.nu);
        for j = 1:m(i)
            z2uMap(jx, ix) = eye(mpc.nu, mpc.nu);
            jx = jx + mpc.nu;
        end
        ix = ix + mpc.nu;
    end
end