function [x_actual,U_total] = control_loop(psi_ref,A,B,z0,params)
    dt = params.dt;
    Nh = params.Nh;
    p = params.p;
    h = params.h;
    quad = quadrotor();
    x_actual = zeros(18,size(psi_ref,2));
    U_total = zeros(4,size(psi_ref,2));
    %psi_x = [X(1:6,1);q;w_x(:);psi_bar_x];
    new_states = unlift(z0);
    for i=1:size(psi_ref,2)-Nh
        y = psi_ref(:,i:i+Nh-1);
        y = y(:);
        U = MPC_optimization(A,B,Nh,z0,y);
        U_total(:,i) = U(1:4,1);
        %f_M = quad.get_force_torque(U(1:4,1));
        ft = U(1,1);
        M = U(2:4,1);
        new_states = quad.evolve(ft,M,new_states,dt,h);
        x_actual(:,i) = new_states;
        psi_x = get_lifted_single_vec(new_states,p);
        z0 = psi_x;
    end
    
end