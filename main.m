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

[A,B,psi_x,psi_y,psi_x_aug] = train(params);
%I should use a new reference trajectory, but, for the sake of testing I
%will use the same as the training
load data2.mat X_ref
load data2.mat X0
%X_ref = references();
psi_ref = zeros(size(psi_x,1),size(X_ref,2));
for i=1:length(X_ref)
    psi = get_lifted_single_vec(X_ref(:,i),params.p);
    psi_ref(:,i) = psi;
end
%[p,v,q,w];
states_0 = X0;
z0 = get_lifted_single_vec(states_0,params.p);
[x_actual,U_total] = control_loop(psi_ref,A,B,z0,params);

%t = linspace(0,params.T,params.T/params.dt);
fig = figure(1);
plot3(X_ref(1,:),X_ref(2,:),X_ref(3,:));
hold on
plot3(x_actual(1,:),x_actual(2,:),x_actual(3,:),'--');
hold off
grid on