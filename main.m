close all;
clc
clear

params.n_trajs = 100;
params.T = 0.1;
params.dt = 0.001;
N = params.T/params.dt;
params.mu = [0;0;0;0]; 
params.sigma = diag([10;10;10;10]);
params.p=3;
params.Nh = 10;
params.h = 5*10^(-4);

[A1,B1,psi_x,psi_y,psi_x_aug] = train(params);
%I should use a new reference trajectory, but, for the sake of testing I
%will use the same as the training
psi_ref = psi_x(:,1:500);
R = eye(3);
states_0 = [zeros(9,1);R(:)];
z0 = get_lifted_single_vec(states_0,params.p);
x_actual = control_loop(psi_ref,A1,B1,z0,params);

%t = linspace(0,params.T,params.T/params.dt);
fig = figure(1);
plot3(psi_x(1,1:N),psi_x(2,1:N),psi_x(3,1:N));
hold on
plot3(x_actual(1,1:N),x_actual(2,1:N),x_actual(3,1:N),'o');
hold off