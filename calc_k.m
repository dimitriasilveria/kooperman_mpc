function K = calc_k(psi_x,psi_y,U)
    psi_x_aug = [psi_x;U];
    M = size(psi_x_aug,2);
    n = size(psi_x,1);
    m = size(U,1);
    %K = zeros(15+9*p,n*m);
    G1 = zeros(n,size(psi_x_aug,1));
    G2 = zeros(size(psi_x_aug,1),size(psi_x_aug,1));
    for i=1:M
        psi = psi_y(:,i);
        psi_a = psi_x_aug(:,i);
        G1 = G1 + (1/M)*(psi*psi_a');
        G2 = G2 + (1/M)*(psi_a*psi_a');
    end
    K = G1*pinv(G2);

end