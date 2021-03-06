function [zLB, zUB] = setBounds(mpc)

zLB = [repmat(mpc.boundX_lb,mpc.predictionHorizon,1);...
    repmat(mpc.boundU_lb,mpc.controlHorizon,1)];
zUB = [repmat(mpc.boundX_ub,mpc.predictionHorizon,1);...
     repmat(mpc.boundU_ub,mpc.controlHorizon,1)];

end