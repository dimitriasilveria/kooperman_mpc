function [psi_y,psi_x] = get_lifted_xy(X,Y,p)
    psi_y = [];
    psi_x = [];
    for i=1:size(X,2)
        R_x= reshape(X(10:18,i),[3,3]);
        w_x = X(7:9,i);
        w_hat_x = R3_so3(w_x);
        h_x = R_x;
        psi_bar_x = [];
        R_y= reshape(Y(10:18,i),[3,3]);
        w_y = Y(7:9,i);
        w_hat_y = R3_so3(w_y);
        h_y = R_y;
        psi_bar_y = [];
        for p_index=1:p
            h_x = h_x*w_hat_x;
            psi_bar_x = [psi_bar_x; h_x(:)];
            h_y = h_y*w_hat_y;
            psi_bar_y = [psi_bar_y; h_y(:)];
        end
        R_t = R_x';
        w_t = w_hat_x';
        psi_x = [psi_x,[X(1:6,i);R_t(:);w_t(:);psi_bar_x]];
        R_t = R_y';
        w_t = w_hat_y';
        psi_y = [psi_y,[Y(1:6,i);R_t(:);w_t(:);psi_bar_y]];
    end
end