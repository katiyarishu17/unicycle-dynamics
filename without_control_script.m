global a b c r v_bar;

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

%%

% Specifying initial conditions and solving the system
op = odeset('RelTol',1e-7,'AbsTol',1e-7);

t_final = 10;

% init = [0;0;0;0;0;0.05]; % longitudinal deviation
init = [0;0;0;0.05;0;0]; % lateral deviation
% init = [0;0;0;-0.07;0;0.08]; % lateral and longitudinal deviation

[t,y] = ode45('without_control',[0 t_final],init,op);

%%

% plotting graphs
figure

subplot(3,2,1)
plot(t,y(:,1))
xlabel('time (t)','Interpreter','latex','FontSize',12)
ylabel('yaw rate $(\dot{\phi})$','Interpreter','latex','FontSize',12)

subplot(3,2,3)
plot(t,y(:,3))
xlabel('time (t)','Interpreter','latex','FontSize',12)
ylabel('roll rate $(\dot{\theta}$)','Interpreter','latex','FontSize',12)

subplot(3,2,5)
plot(t,y(:,5))
xlabel('time (t)','Interpreter','latex','FontSize',12)
ylabel('pitch rate $(\dot{\psi})$','Interpreter','latex','FontSize',12)

subplot(3,2,2)
plot(t,y(:,2))
xlabel('time (t)','Interpreter','latex','FontSize',12)
ylabel('yaw $(\phi)$','Interpreter','latex','FontSize',12)

subplot(3,2,4)
plot(t,y(:,4))
xlabel('time (t)','Interpreter','latex','FontSize',12)
ylabel('roll $(\theta$)','Interpreter','latex','FontSize',12)

subplot(3,2,6)
plot(t,y(:,6))
xlabel('time (t)','Interpreter','latex','FontSize',12)
ylabel('pitch $(\psi)$','Interpreter','latex','FontSize',12)

%%

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

    distance = (t(i)-t(i-1))*v_bar;

    z(i,1) = z(i-1,1) - 0.26*sin(z(i-1,5))*sin(z(i-1,4)) + distance*cos(z(i-1,4));
    z(i,2) = z(i-1,2) + 0.26*sin(z(i-1,5))*cos(z(i-1,4)) + distance*sin(z(i-1,4));

    z(i,1) = z(i,1) + 0.26*sin(z(i,5))*sin(z(i,4));
    z(i,2) = z(i,2) - 0.26*sin(z(i,5))*cos(z(i,4));
    z(i,3) = 0.26*cos(z(i,5));

%     z(i,1) = z(i-1,1) + distance*cos(z(i-1,4));
%     z(i,2) = z(i-1,2) + distance*sin(z(i-1,4));

%     z(i,1) = z(i,1) + 0.26*sin(z(i,5))*sin(z(i,4));
%     z(i,2) = z(i,2) + 0.26*sin(z(i-1,5))*cos(z(i-1,4)) - 0.26*sin(z(i,5))*cos(z(i,4));
%     z(i,3) = 0.26*cos(z(i,5));

    z(i,7) = z(i-1,7) + distance/0.26;
end

t_range = 0:0.04:t_final;
z_range = interp1(t,z,t_range,'spline');
data = [(0:length(t_range)-1)',z_range];
data = round(data,3);

writematrix(data)