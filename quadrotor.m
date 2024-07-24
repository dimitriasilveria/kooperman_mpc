classdef quadrotor
    properties (Constant)
        m = 4.34;
        Ixx = 0.082;
        Iyy = 0.0845;
        Izz = 0.1377;
        d = 0.315;
        c_tau = 8*10^(-4);
        J = diag([0.082,0.0845,0.1377]);
        g = [0 0 -9.8]';
        I3 = [0 0 1]'; 
        Map = [1 1 1 1; 0 -0.315 0 0.315; 0.315 0 -0.315 0;-8*10^(-4) 8*10^(-4) -8*10^(-4) 8*10^(-4)];

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
        function y_next = evolve(obj,ft,M,y_prev,dt,h)
            %y_prev = [x;v;w;R(:)];
            R = reshape(y_prev(10:18),[3,3]);
            N = dt/h;
            for i=1:N
                k1 = obj.ode_system(y_prev(1:9),ft,M,R);
                k2 = obj.ode_system(y_prev(1:9) + 0.5*h*k1,ft,M,R);
                k3 = obj.ode_system(y_prev(1:9) + 0.5*h*k2,ft,M,R);
                k4 = obj.ode_system(y_prev(1:9) + k3*h,ft,M,R);
                y_next = y_prev(1:9,1) + (h/6)*(k1 + 2*k2 + 2*k3 + k4);
                y_prev = y_next;
                w = y_next(7:9,1);
                R = R*expm(R3_so3(w));
            end
            w = y_next(7:9);
            R = R*expm(R3_so3(w));
            y_next = [y_next;R(:)];
        end
        function f_M = get_force_torque(obj,U)
            f_M = obj.Map*U;
        end
    end
end
