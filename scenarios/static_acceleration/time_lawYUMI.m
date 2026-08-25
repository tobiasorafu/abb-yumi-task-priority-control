clc; close all; clear all;

%% Path in PARAMETER space (lambda), not Cartesian space
lambda_i = 0;
lambda_f = 6*pi;              % 3 full laps around the ellipse (matches Ttot in main code)
L = lambda_f - lambda_i;      % "distance" is now an angle, not a Euclidean length

a_max = 2;                    % max angular acceleration of the path parameter [rad/s^2]
v_max = sqrt(L*a_max);        % triangular (bang-bang) profile -> no coast phase
Tf = (L*a_max + v_max^2)/(a_max*v_max);
Ts = 0.5*Tf;

step = 0.001;
t = 0:step:Tf;
N = size(t,2);

sigma   = zeros(1,N);
dsigma  = zeros(1,N);
ddsigma = zeros(1,N);

d1 = find(t>Ts); d1 = d1(1)-1;

% acceleration phase
sigma(1:d1)   = (a_max.*t(1:d1).^2)/2;
dsigma(1:d1)  = a_max.*t(1:d1);
ddsigma(1:d1) = a_max*ones(1,d1);

% deceleration phase
sigma(d1+1:end)   = -0.5*a_max*(t(d1+1:end)-Tf).^2 + v_max*Tf - v_max^2/a_max;
dsigma(d1+1:end)  = a_max*(Tf - t(d1+1:end));
ddsigma(d1+1:end) = -a_max*ones(1,N-d1);

%% Map sigma directly to the path parameter
lambda   = lambda_i + sigma;
dlambda  = dsigma;
ddlambda = ddsigma;

%% Evaluate Cartesian trajectory via chain rule through Task_yumi
px = zeros(1,N); py = zeros(1,N); pz = zeros(1,N);
dpx = zeros(1,N); dpy = zeros(1,N); dpz = zeros(1,N);
ddpx = zeros(1,N); ddpy = zeros(1,N); ddpz = zeros(1,N);

for k = 1:N
    [p, dp_dlambda, ddp_dlambda] = Task_yumi(lambda(k));
    px(k) = p(1); py(k) = p(2); pz(k) = p(3);
    dp = dp_dlambda * dlambda(k);
    dpx(k) = dp(1); dpy(k) = dp(2); dpz(k) = dp(3);
    ddp = ddp_dlambda*(dlambda(k))^2 + dp_dlambda*ddlambda(k);
    ddpx(k) = ddp(1); ddpy(k) = ddp(2); ddpz(k) = ddp(3);
end

%% Plots
figure
subplot(3,1,1); plot(t,sigma,'b','LineWidth',2); ylabel('$\sigma$','Interpreter','latex','FontSize',18)
subplot(3,1,2); plot(t,dsigma,'b','LineWidth',2); ylabel('$\dot{\sigma}$','Interpreter','latex','FontSize',18)
subplot(3,1,3); plot(t,ddsigma,'b','LineWidth',2); ylabel('$\ddot{\sigma}$','Interpreter','latex','FontSize',18); xlabel('time [s]')

figure
subplot(3,1,1); plot(t,px,'b',t,py,'r',t,pz,'g','LineWidth',2); title('Desired position vs time'); legend('X','Y','Z')
subplot(3,1,2); plot(t,dpx,'b',t,dpy,'r',t,dpz,'g','LineWidth',2); title('Desired velocity vs time')
subplot(3,1,3); plot(t,ddpx,'b',t,ddpy,'r',t,ddpz,'g','LineWidth',2); title('Desired acceleration vs time')
