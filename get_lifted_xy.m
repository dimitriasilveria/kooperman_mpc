function [psi_y,psi_x] = get_lifted_xy(X,Y,p)
    psi_y = [];
    psi_x = [];
    for i=1:size(X,2)
        R_x= reshape(X(7:15,i),[3,3]);
        w_x = X(16:18,i);
        w_hat_x = R3_so3(w_x);
        h_x = R_x;
        psi_bar_x = [];
        R_y= reshape(Y(7:15,i),[3,3]);
        w_y = Y(16:18,i);
        w_hat_y = R3_so3(w_y);
        h_y = R_y;
        psi_bar_y = [];
        for p_index=1:p
            h_x = h_x*w_hat_x;
            psi_bar_x = [psi_bar_x; h_x(:)];
            h_y = h_y*w_hat_y;
            psi_bar_y = [psi_bar_y; h_y(:)];
        end
        qx = RotToQuat(R_x);
        w_t = w_hat_x';
        psi_x = [psi_x,[X(1:6,i);qx;w_x;psi_bar_x]];
        qy = RotToQuat(R_y);
        w_t = w_hat_y';
        psi_y = [psi_y,[Y(1:6,i);qy;w_y;psi_bar_y]];
    end
end