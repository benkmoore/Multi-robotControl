function [A, b_tilde] = setEqConstraints(mpc, A_kron, B_kron)

con = zeros(mpc.nx * mpc.predictionHorizon,size(B_kron,2));
con(1:mpc.nx,:) = B_kron;
for k=1:mpc.predictionHorizon-1
  con(k*mpc.nx+1:(k+1)*mpc.nx,:) = A_kron * con((k-1)*mpc.nx+1:k*mpc.nx,:);
end
A12 = zeros(mpc.predictionHorizon * mpc.nx, mpc.controlHorizon * mpc.nu);
for k=1:mpc.controlHorizon
    A12((k-1)*mpc.nx+1:end,(k-1)*size(B_kron,2)+1:k*size(B_kron,2)) =  con(1:end-(k-1)*mpc.nx,:); 
end
A11 = eye(mpc.nx*mpc.predictionHorizon,mpc.nx*mpc.predictionHorizon);
A = sparse([A11 -A12]);

b_tilde = zeros(mpc.nx*mpc.predictionHorizon,mpc.nx);
b_tilde(1:mpc.nx,:) = A_kron;
for k=1:mpc.predictionHorizon-1
  b_tilde(k*mpc.nx+1:(k+1)*mpc.nx,:) = A_kron * b_tilde((k-1)*mpc.nx+1:k*mpc.nx,:);
end
b_tilde = sparse(b_tilde);
end