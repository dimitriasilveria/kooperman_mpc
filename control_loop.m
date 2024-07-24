function x_actual = control_loop(psi_ref,A,B,z0,params)
    dt = params.dt;
    Nh = params.Nh;
    p = params.p;
    h = params.h;
    quad = quadrotor();
    x_actual = zeros(18,size(psi_ref,2));
    new_states = z0; 
    for i=1:size(psi_ref,2)-Nh
        y = psi_ref(:,i:i+Nh-1);
        y = y(:);
        U = MPC_optimization(A,B,Nh,z0,y);
        f_M = quad.get_force_torque(U(1:4,1));
        ft = f_M(1,1);
        M = f_M(2:4,1);
        new_states = quad.evolve(ft,M,new_states,dt,h);
        x_actual(:,i) = new_states;
        psi_x = get_lifted_single_vec(new_states,p);
        z0 = psi_x;
    end
    
end