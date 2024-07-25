function U = MPC_optimization(A,B,Nh,z0,y)
    [Q_hat,R_hat,Aqp,Bqp,A_ineq,b_ineq] = get_MPC_matrices(A,B,Nh);
    H = 2*(Bqp'*Q_hat*Bqp+R_hat);
    %H=(H+H')/2;
    G = 2*Bqp'*Q_hat*(Aqp*z0-y);
    options = optimoptions('quadprog','MaxIterations',1e4);
    U_h = quadprog(H,G,A_ineq,b_ineq,[],[],[],[],[],options);
    U = U_h(:,1);
end