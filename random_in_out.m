function [X,Y,U] = random_in_out(n_trajs,T,dt,mu,sigma,h)
    quad = quadrotor();
    N = int32(T/dt);
%     psi = zeros(int(24+9*p),n_trajs,N); %lifted states
%     psi_bar = zeros(int(9*p),n_trajs,N); %observable functions
%     psi_a = zeros(int(9*p+4),n_trajs,N); %observable + inputs
    X = [];
    Y = [];
    U = [];
    %y_prev = [x;v;w;R(:)];
    for i=1:n_trajs
        R = eye(3);
        states = [zeros(9,1);R(:)];
        new_states = states;
        U_rand = mvnrnd(mu,sigma,1)';
        u = [];
        %sampling the inputs
        for t=1:N
            u = [u,U_rand];
            f_M = quad.get_force_torque(U_rand);
            ft = f_M(1,1);
            M = f_M(2:4,1);
            X = [X,new_states];
            new_states = quad.evolve(ft,M,new_states,dt,h); 
            Y = [Y,new_states];
%             x(:,:,i,t+1) = y(:,:,i,t);
%             R= y(4:6,i,t);
%             w = y(3,i,t);
%             h = R;
%             h_bar = [];
%             for p_index=1:p
%                 h = h*R3_so3(w);
%                 h_bar = [h_bar; h(:)];
%             end
%             psi_bar(:,i,t) = h_bar;
%             w_hat = R3_so3(w);
%             R_t = R';
%             w_hat_t = w_hat';
%             psi(:,i,t) = [y(1,:,i,t)';y(2,:,i,t)';R_t(:);w_hat_t(:);psi_bar(:,i,t)];
%             psi_a(:,i,t) = [psi_bar(:,i,t);random_in];
        end
        x = states(1:end-18,1);
        y = states(19:end,1);

        % du = u(1:end-4);
        U = [U,u];
    end
end