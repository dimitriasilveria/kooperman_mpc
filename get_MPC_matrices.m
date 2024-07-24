function [Q_hat,R_hat,Aqp,Bqp,A_ineq,b_ineq] = get_MPC_matrices(A,B,Nh)
    n = size(A,2);
    Q_hat = []; R_hat = [];
    A_ineq = []; b_ineq = [];
    
    Qx = diag([1e6;1e6;1e6]);
    Qv = diag([1e5;1e5;1e6]);
    Qa = 1e5*eye(3);
    Qw = 1e5*eye(3);
    Q_i = 0*eye(size(A,2));
    Q_i(1:12,1:12) = blkdiag(Qx, Qv, Qa, Qw);
    
    R_i = diag([1e1;1e0;1e0;1e0]);

    Aqp = [];
    Bqp = [];
    for i=1:Nh
        Aqp = [Aqp; A^i];
        if i > 1
            B_c = zeros((i-1)*size(B,1),size(B,2));
        else
            B_c = [];
        end
        for j=i:Nh
            B_c =[B_c;A^(i-1)*B]; 
        end
        Bqp = [Bqp,B_c];
    
        Q_hat = blkdiag(Q_hat, Q_i);
        R_hat = blkdiag(R_hat, R_i);
    
    
        %A_ineq_i = repmat([-1;1],4,1);
        A_ineq_i = kron(eye(4),[-1;1]);
        A_ineq = blkdiag(A_ineq, A_ineq_i);
        
        % set lower and upper bounds for control inputs
        u_lb = -10.*[0;0.05;0.05;0.05];
        u_ub = 10.*[2;0.05;0.05;0.05];
        
    %     b_ineq_i = [-u_lb; u_ub];
    %     b_ineq_i = repmat(b_ineq_i,4,1);
        
        b_ineq_i = [-u_lb, u_ub]';
        b_ineq_i = b_ineq_i(:);
        b_ineq = [b_ineq; b_ineq_i];

    
    end

Q_hat(end-n+1:end,end-n+1:end) = Q_i;

end