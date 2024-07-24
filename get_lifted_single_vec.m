function psi_x = get_lifted_single_vec(X,p)
    R_x= reshape(X(7:15,1),[3,3]);
    w_x = X(16:18,1);
    w_hat_x = R3_so3(w_x);
    h_x = R_x;
    psi_bar_x = [];
    for p_index=1:p
        h_x = h_x*w_hat_x;
        psi_bar_x = [psi_bar_x; h_x(:)];
    end
    q = RotToQuat(R_x);
    w_t = w_hat_x';
    psi_x = [X(1:6,1);q;w_x(:);psi_bar_x];
end