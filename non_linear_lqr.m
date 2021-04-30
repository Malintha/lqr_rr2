clear;
addpath('./mr/')
addpath('./plots/')

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
Tf = 3;

% calculate the cartesian trajectory
st_tr = trvec2tform([start 0]);
target_tr = trvec2tform([target 0]);
traj = CartesianTrajectory(st_tr, target_tr, Tf, Tf/dt, 5);

% system matrices for LQR with some modelling noise

B = @(theta1,theta2) [-L1*sin(theta1)-L2*sin(theta1+theta2) -L2*sin(theta1+theta2); ...
                    L1*cos(theta1)+L2*cos(theta1+theta2) L2*cos(theta1+theta2);];

A = ones(2);               
% cost matrices Q: state, R: control

Q = eye(2)*1000;
% use 0.1 to increase the aggressiveness of control  
R = [0.1 0;
    0 0.1];

% initial conditions for the system
% q = qinit;
x0 = [start'; qinit(1); qinit(2)];

% matrices for plotting
xs = [];
es = [];
xbars = [];

% initialize inverse kinematics
weights = [0, 0, 0, 1, 1, 0];
ik = inverseKinematics('RigidBodyTree', robot);

% initialize generalilzed inverse kinematics (too slow thus retreated to the old one)
% gik = generalizedInverseKinematics('RigidBodyTree',robot, ... 
%                             'ConstraintInputs',{'position','joint'});
% jointConst = constraintJointBounds(robot);
% jointConst.Bounds = [theta_min pi-theta_min; theta_min pi-theta_min];
% posConst = constraintPositionTarget('tool');

ddt = 0.01;
for i=2:length(traj)
    if i<length(traj)
        [~, x_bar] = TransToRp(traj{i});
    else
        [~, x_bar] = TransToRp(traj{end});
    end

%     get the theta_bar that needs the robot to be stabilized around
    q_bar = ik('tool', trvec2tform([x0(1:2)' 0]), weights, x0(3:4));

%     posConst.TargetPosition = x_bar;
%     q_bar = gik(x0(3:4), posConst, jointConst);
    
%     linearized system around the fixed point 

    Bt = B(q_bar(1), q_bar(2));

%     get the LQR gain matrix
    k = lqr(A, Bt, Q, R);
    
% solve the system with kinematics
% use state as [x y theta1 theta2]
    tspan = (0:ddt:dt);
    x_bar = x_bar(1:2);
    [~, xt] = ode45(@(t,xt)non_sys(xt, x_bar, k), tspan, x0);
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

% xlim([-1.2 1.2]);
% ylim([-1.2 1.2]);
title('Angular & Cartesian Fixed Points Stabilization');
hold on

% plot the goal and trajectory
scatter(target(1),target(2),40,'*b');
scatter(start(1),start(2),40,'*g');
plot(xbars(:,1),xbars(:,2),'-g','LineWidth',1);

framesPerSecond = length(xs)/Tf;
r = rateControl(framesPerSecond);

for i = 1:length(qs)
    f1 = show(robot,qs(i,:)','PreservePlot',false);
    h=findall(f1); %finding all objects in figure
    hlines=h.findobj('Type','Line'); %finding line object 
    %editing line object properties
    n=size(hlines);
    for j=1:2
        hlines(j).LineWidth = 3; %chanding line width
        hlines(j).Color=[1 0 0.5];%changing line color
        hlines(j).Marker='o';%changing marker type
        hlines(j).MarkerSize=5; %changing marker size
    end
    drawnow

    %% save to gif
%     filename = './plots/added_noise1.gif';
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

% visualize the plots
visualize_plots(ddt, dt, es, xs, xbars);

