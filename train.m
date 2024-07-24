function [A,B,psi_x,psi_y,psi_x_aug] =train(params)
    T = params.T;
    dt = params.dt;
    mu = params.mu;
    sigma = params.sigma;
    p = params.p;
    h = params.h;
    n_trajs = params.n_trajs;
    [X,Y,U] = random_in_out(n_trajs,T,dt,mu,sigma,h);
    [psi_y,psi_x] = get_lifted_xy(X,Y,p);
    K = calc_k(psi_x,psi_y,U);
    
    m = size(U,1);
    n = size(psi_x,1)-9*p;
    
    A = K(:,1:end-m);
    B = K(:,end-m+1:end);
    C = [eye(n), zeros(n,9*p)];
    psi_x_aug = [psi_x;U];
    x = C*psi_x;
    
end