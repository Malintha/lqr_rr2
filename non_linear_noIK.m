clear;
addpath('./mr/')

% L1, L2: arm lengths
L1 = 0.5;
L2 = 0.5;
theta_min = 0.1;

robot = get2RRobot(L1, L2);

% start and goal positions for the end effector
qinit = [theta_min ; theta_min];

start = [L1*cos(theta_min) + L2*cos(2*theta_min) ...
        L1*sin(theta_min) + L2*sin(2*theta_min)];
target = [-0.5 0.6];

% times for cartesian trajectory generation
dt = 0.1;
Tf = 5;
% calculate the cartesian trajectory
st_tr = trvec2tform([start 0]);
target_tr = trvec2tform([target 0]);
traj = CartesianTrajectory(st_tr, target_tr, Tf, Tf/dt, 5);

% system matrices for LQR
A = eye(2);
B = @(theta1,theta2) [-L1*sin(theta1)-L2*sin(theta1+theta2) -L2*sin(theta1+theta2); ...
                    L1*cos(theta1)+L2*cos(theta1+theta2) L2*cos(theta1+theta2);];
                
% cost matrices
Q = eye(2)*100;
R = [10 0;
    0 10];

% initial conditions for the system
q = qinit;
x0 = [start'; q(1); q(2)];

% matrices for plotting
xs = [];
es = [];
xbars = [];
ddt = 0.01;
for i=2:length(traj)
    if i<length(traj)
        [~, x_bar] = TransToRp(traj{i});
    else
        [~, x_bar] = TransToRp(traj{end});
    end
    x_bar = x_bar(1:2);
    
%     linearized system around the current state/initial condition
    Bt = B(x0(3), x0(4));
    
%     get the LQR gain matrix
    k = lqr(A, Bt, Q, R);
    
% solve the system with kinematics
% use state as [x y theta1 theta2] to progress the system
    tspan = (0:ddt:dt);
    [~, xt] = ode45(@(t,xt)non_sys_noIK(xt, x_bar, k), tspan, x0);
    x0 = xt(end,:)';
    
    % plotting data
    es = [es; xt(:,1:2)-x_bar(1:2)'];
    xs = [xs; xt];
    xbars = [xbars; x_bar'];
end

%% animate the robot
% Note: Animating is slower than the actual rate when the time resolution 
% is too high, especially when the results from the ODE integration is passed.
qs = xs(:,3:4);
figure('Renderer', 'painters', 'Position', [1500 100 600 600]);
f1 = show(robot,qs(1,:)');
view(2);
ax = gca;
ax.Projection = 'orthographic';
xlim([-1.2 1.2]);
ylim([-1.2 1.2]);
title('TVLQR with angular & cartesian fixed points');
hold on

% plot the goal and trajectory
scatter(target(1),target(2),40,'*b');
scatter(start(1),start(2),40,'*g');
plot(xbars(:,1),xbars(:,2),'-g','LineWidth',1);

framesPerSecond = length(qs)/Tf;
r = rateControl(framesPerSecond);
tic
for i = 1:length(qs)
    f1 = show(robot,qs(i,:)','PreservePlot',false);
%     drawnow
    h=findall(f1); %finding all objects in figure
    hlines=h.findobj('Type','Line'); %finding line object 
    %editing line object properties
    for j=1:2
        hlines(j).LineWidth = 3; %chanding line width
        hlines(j).Color=[1 0 0.5];%changing line color
        hlines(j).Marker='o';%changing marker type
        hlines(j).MarkerSize=5; %changing marker size
    end

% %     save to gif
%     filename = 'non_linear_noIK.gif';
%     frame = getframe(1);
%     im = frame2im(frame);
%     [imind,cm] = rgb2ind(im,256);
%     if i == 1 
%       imwrite(imind,cm,filename,'gif', 'Loopcount',inf);
%     else
%       imwrite(imind,cm,filename,'gif','Writemode','append', 'DelayTime', dt);
%     end

    waitfor(r);
end

hold off
toc
reset(r)

% visualize the plots
visualize_plots(ddt, dt, es, xs, xbars);

