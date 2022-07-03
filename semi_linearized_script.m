global a b c r v_bar b_bar b1 b2 b3 b4 b5 b6;

% Setting parameter values
J_T_frame = 1;
J_P_frame = 0.3;
J_T_tt = 2.2;
J_P_tt = 4;
r = 0.1;
v_bar = 2.7;

a = (r+1)^2+J_T_frame+J_T_tt;
b = J_P_tt;
c = J_P_frame+J_P_tt;
b_bar = v_bar*b/c;

%%

% Setting alpha_i denoted as a_i
a1 = 0.6551;
a2 = 0.1626;
a3 = 0.1190;
a4 = 0.4984;
a5 = 0.9597;
a6 = 0.3404;

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

t_final = 40;

% init = [0;0;0;0;0;0.12;0;0]; % longitudinal deviation
% init = [0;0;0;-0.09;0;0;0;0]; % lateral deviation (uncontrolled)
% init = [0;0;0;0.01;0;0;0;0]; % lateral deviation (controlled)
% init = [0;0;0;-0.07;0;0.08;0;0]; % lateral and longitudinal deviation
init = [0;1;0;0;0;0;0;0]; % new heading direction

init(1) = -b*(b1*init(3)+b2*init(4)+b3*init(7))/c;

[t,y] = ode45('semi_linearized',[0 t_final],init,op);

%%

% plotting graphs
figure

subplot(3,2,1)
plot(t,y(:,2))
xlabel('time (t)','Interpreter','latex','FontSize',12)
ylabel('yaw $(\phi)$','Interpreter','latex','FontSize',12)

subplot(3,2,3)
plot(t,y(:,4))
xlabel('time (t)','Interpreter','latex','FontSize',12)
ylabel('roll $(\theta$)','Interpreter','latex','FontSize',12)

subplot(3,2,5)
plot(t,y(:,6))
xlabel('time (t)','Interpreter','latex','FontSize',12)
ylabel('pitch $(\psi)$','Interpreter','latex','FontSize',12)

subplot(3,2,2)
plot(t,y(:,7))
xlabel('time (t)','Interpreter','latex','FontSize',12)
ylabel('$\xi$','Interpreter','latex','FontSize',12)

subplot(3,2,4)
plot(t,y(:,8))
xlabel('time (t)','Interpreter','latex','FontSize',12)
ylabel('$u$','Interpreter','latex','FontSize',12)

%%

% % exporting data for animation
% len = length(t);
% z = zeros(len,7);
% z(1,:) = [0,0,y(1,2),y(1,4),y(1,6),0,y(1,7)];
% 
% for i=2:len
%     distance = (t(i)-t(i-1))*(v_bar+y(i-1,8));
% 
%     z(i,1) = z(i-1,1) + distance*cos(z(i-1,3));
%     z(i,2) = z(i-1,2) + distance*sin(z(i-1,3));
% 
%     z(i,3) = y(i,2);
%     z(i,4) = y(i,4);
%     z(i,5) = y(i,6);
% 
%     z(i,6) = z(i-1,6) + distance/0.26;
% 
%     z(i,7) = y(i,7);
% end
% 
% t_range = 0:0.04:t_final;
% y_range = interp1(t,z,t_range,'spline');
% data = [(0:length(t_range)-1)',y_range];
% data = round(data,3);
% % final_data(:,7) = -final_data(:,7);
% 
% writematrix(data)

% exporting data for animation
len = length(t);
z = zeros(len,8);

z(1,4) = y(1,2);
z(1,5) = y(1,4);
z(1,6) = y(1,6);
z(1,1) = 0 + 0.26*sin(z(1,5))*sin(z(1,4));
z(1,2) = 0 - 0.26*sin(z(1,5))*cos(z(1,4));
z(1,3) = 0.26*cos(z(1,5));

for i=2:len
    z(i,4) = y(i,2);
    z(i,5) = y(i,4);
    z(i,6) = y(i,6);

    distance = (t(i)-t(i-1))*(v_bar+y(i-1,8));

    z(i,1) = z(i-1,1) - 0.26*sin(z(i-1,5))*sin(z(i-1,4)) + distance*cos(z(i-1,4));
    z(i,2) = z(i-1,2) + 0.26*sin(z(i-1,5))*cos(z(i-1,4)) + distance*sin(z(i-1,4));

    z(i,1) = z(i,1) + 0.26*sin(z(i,5))*sin(z(i,4));
    z(i,2) = z(i,2) - 0.26*sin(z(i,5))*cos(z(i,4));
    z(i,3) = 0.26*cos(z(i,5));

    z(i,7) = z(i-1,7) + distance/0.26;
    z(i,8) = y(i,7);
end

t_range = 0:0.04:t_final;
z_range = interp1(t,z,t_range,'spline');
data = [(0:length(t_range)-1)',z_range];
data = round(data,3);

writematrix(data)