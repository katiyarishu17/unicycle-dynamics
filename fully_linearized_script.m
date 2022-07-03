global a b c r v_bar b_bar b1 b2 b3 b4 b5 b6;

% Setting parameter values
J_T_frame = 1;
J_P_frame = 0.3;
J_T_tt = 2.2;
J_P_tt = 4;
r = 0.1;
v_bar = 2.3;

a = (r+1)^2+J_T_frame+J_T_tt;
b = J_P_tt;
c = J_P_frame+J_P_tt;
b_bar = v_bar*b/c;

%%

% Setting alpha_i denoted as a_i
a1 = 0.4456;
a2 = 0.6463;
a3 = 0.7094;
a4 = 0.7547;
a5 = 0.2760;
a6 = 0.6797;

%%

% Variational equations in matrix form
A = [0 (r+1)/a 0 0 0 0 (r+1)*b_bar/a 0 0 0 0 0
     1 0 0 0 0 0 0 0 0 0 0 0
     0 0 0 1/a 0 0 0 -(r+1)/a 0 0 0 0
     0 0 1 0 0 0 0 0 0 0 0 0
     0 0 0 0 0 0 1 0 0 0 0 0
     0 0 0 0 0 0 0 1 0 0 0 0
     0 0 0 0 a3/a5 0 0 0 (r+1)*b_bar/a5 0 0 0
     0 0 0 0 0 a4/a6 0 0 0 0 -(r+1)/a6 0
     0 -a1/a 0 0 0 0 0 0 0 (r+1)/a 0 0
     0 0 0 0 0 0 0 0 1 0 0 0
     0 0 0 -a2/a 0 0 0 0 0 0 0 1/a
     0 0 0 0 0 0 0 0 0 0 1 0];

[V,D] = eig(A);
D = real(diag(D));
[D,N] = sort(D);

N = N(1:6);
V = V(:,N);

V1 = V(1:6,:);
V2 = V(7:12,:);

U = real(V2*inv(V1));

b1 = U(1,1);
b2 = U(1,2);
b3 = U(1,5);
b4 = U(2,3);
b5 = U(2,4);
b6 = U(2,6);

%%

% Verifying the feedback control equations
B = [(r+1)*b_bar*b1/a (r+1)*(1+b_bar*b2)/a 0 0 (r+1)*b_bar*b3/a 0
     1 0 0 0 0 0
     0 0 -(r+1)*b4/a (1-(r+1)*b5)/a 0 -(r+1)*b6/a
     0 0 1 0 0 0
     b1 b2 0 0 b3 0
     0 0 b4 b5 0 b6];

[v,d] = eig(B);
d = real(diag(d));
[d,n] = sort(d);

%%

% Specifying initial conditions and solving the system
op = odeset('RelTol',1e-7,'AbsTol',1e-7);

t_final = 20;

init = [0;0;0;-0.09;0;0]; % longitudinal deviation
% init = [0;0.11;0;0;0;0]; % lateral deviation
% init = [0;-0.07;0;0.08;0;0]; % lateral and longitudinal deviation

[t,y] = ode45('fully_linearized',[0 t_final],init,op);

%%

% plotting graphs
figure

subplot(2,2,1)
plot(t,y(:,2))
xlabel('time (t)','Interpreter','latex','FontSize',12)
ylabel('roll $(\theta)$','Interpreter','latex','FontSize',12)

subplot(2,2,2)
plot(t,y(:,4))
xlabel('time (t)','Interpreter','latex','FontSize',12)
ylabel('pitch $(\psi$)','Interpreter','latex','FontSize',12)

subplot(2,2,3)
plot(t,y(:,5))
xlabel('time (t)','Interpreter','latex','FontSize',12)
ylabel('$\xi$','Interpreter','latex','FontSize',12)

subplot(2,2,4)
plot(t,y(:,6))
xlabel('time (t)','Interpreter','latex','FontSize',12)
ylabel('$u$','Interpreter','latex','FontSize',12)
