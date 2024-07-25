classdef quadrotor
    properties (Constant)
        m = 0.18;
        Ixx = 0.082;
        Iyy = 0.0845;
        Izz = 0.1377;
        d = 0.315;
        c_tau = 8*10^(-4);
        J = [0.00025,   0,          2.55e-6;
            0,         0.000232,   0;
            2.55e-6,   0,          0.0003738];
        g = [0 0 9.81]';
        I3 = [0 0 1]'; 
        Map = [1 1 1 1; 0 -0.315 0 0.315; 0.315 0 -0.315 0;-8*10^(-4) 8*10^(-4) -8*10^(-4) 8*10^(-4)];
        arm_length = 0.086;
        minF = 0.0;
        maxF = 2.0*0.18*9.81;

    end
    methods
        function dydt = ode_system(obj,y,ft,M,R)
            %ra = y(1,:)';
            w = y(7:9);
            dydt = zeros(9,1);
            dydt(1:3,1) = y(4:6);
            dydt(4:6,1) = R*(obj.I3*ft/obj.m)-obj.g; 
            dydt(7:9,1)= obj.J\(M - cross(w,obj.J*w));
        
        end
        function y = evolve(obj,ft,M,y_prev,dt,h)
            A = [0.25,                      0, -0.5/obj.arm_length;
                 0.25,  0.5/obj.arm_length,                      0;
                 0.25,                      0,  0.5/obj.arm_length;
                 0.25, -0.5/obj.arm_length,                      0];

            prop_thrusts = A*[ft;M(1:2)]; % Not using moment about Z-axis for limits
            prop_thrusts_clamped = max(min(prop_thrusts, obj.maxF/4), obj.minF/4);
            
            B = [                 1,                 1,                 1,                  1;
                                  0, obj.arm_length,                 0, -obj.arm_length;
                 -obj.arm_length,                 0, obj.arm_length,                 0];
            ft= B(1,:)*prop_thrusts_clamped;
            M = [B(2:3,:)*prop_thrusts_clamped; M(3)];

            %y_prev = [x;v;w;R(:)];
            R = reshape(y_prev(7:15),[3,3]);
            N = dt/h;
            for i=1:N
                k1 = obj.ode_system(y_prev(1:9),ft,M,R);
                k2 = obj.ode_system(y_prev(1:9) + 0.5*h*k1,ft,M,R);
                k3 = obj.ode_system(y_prev(1:9) + 0.5*h*k2,ft,M,R);
                k4 = obj.ode_system(y_prev(1:9) + k3*h,ft,M,R);
                y_next = y_prev(1:9,1) + (h/6)*(k1 + 2*k2 + 2*k3 + k4);
                y_prev = y_next;
                w = y_next(7:9,1);
                R = R*expm(R3_so3(w*h));
            end
            w = y_next(7:9,1);
            R = R*expm(R3_so3(w*dt));
            y = [y_next(1:6,1);R(:);w];
        end
        function f_M = get_force_torque(obj,U)
            f_M = obj.Map*U;
        end
    end
end
