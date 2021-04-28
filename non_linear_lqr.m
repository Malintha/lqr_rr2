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
target = [-0.5 0.8];

% times for cartesian trajectory generation
dt = 0.1;
Tf = 3;
% calculate the cartesian trajectory
st_tr = trvec2tform([start 0]);
target_tr = trvec2tform([target 0]);
traj = CartesianTrajectory(st_tr, target_tr, 5, 5/dt, 5);

% system matrices for LQR
A = @(theta1,theta2) [1 0 (-L1*sin(theta1)-L2*sin(theta1+theta2)) (-L2*sin(theta1+theta2)); ... 
                     0 1 (L1*cos(theta1)+L2*cos(theta1+theta2)) (L2*cos(theta1+theta2)); ...
                     0 0 1 0; ... 
                     0 0 0 1];
                
B = @(theta1,theta2) [-L1*sin(theta1)-L2*sin(theta1+theta2) -L2*sin(theta1+theta2); ...
                    L1*cos(theta1)+L2*cos(theta1+theta2) L2*cos(theta1+theta2);
                    1 0; 
                    0 1];
                
% cost matrices
Q = [10 0 0 0; 
     0 10 0 0;
     0 0 1000 0
     0 0 0 1000];
 
R = [0.1 0;
    0 0.1];

% initial conditions for the system
q = qinit;
x0 = [start'; q(1); q(2)];

% matrices for plotting
xs = [];
es = [];

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

    x_bar = [x_bar(1:2); q_bar];
    
%     linearized system around the fixed point 
    At = A(q_bar(1), q_bar(2));
    Bt = B(q_bar(1), q_bar(2));
    
%     get the LQR gain matrix
    k = lqr(At, Bt, Q, R);
    
% solve the system with kinematics
% use state as [x y theta1 theta2]
    tspan = (0:ddt:dt);
    [~, xt] = ode45(@(t,xt)non_sys(xt, x_bar, k), tspan, x0);
    x0 = xt(end,:)';
    
    % plotting data
    es = [es; xt(:,1:2)-x_bar(1:2)'];
    xs = [xs; xt];
end

% animate the robot
% qs = xs(:,3:4);
% figure
% f1 = show(robot,qs(1,:)');
% view(2);
% ax = gca;
% ax.Projection = 'orthographic';
% hold on
% framesPerSecond = 1/ddt;
% r = rateControl(1000);
% 
% for i = 1:length(qs)
%     f1 = show(robot,qs(i,:)','PreservePlot',false);
%     h=findall(f1); %finding all objects in figure
%     hlines=h.findobj('Type','Line'); %finding line object 
%     %editing line object properties
%     n=size(hlines);
%     for j=1:2
%         hlines(j).LineWidth = 3; %chanding line width
%         hlines(j).Color=[1 0 0.5];%changing line color
%         hlines(j).Marker='o';%changing marker type
%         hlines(j).MarkerSize=5; %changing marker size
%     end
%     drawnow
% %     filename = 'added_noise1.gif';
% %     frame = getframe(1);
% %     im = frame2im(frame);
% %     [imind,cm] = rgb2ind(im,256);
% %     if i == 1 
% %       imwrite(imind,cm,filename,'gif', 'Loopcount',inf);
% %     else
% %       imwrite(imind,cm,filename,'gif','Writemode','append', 'DelayTime', dt);
% %     end
%     waitfor(r);
% end
% hold off

% visualize the plots
% fprintf('Total control: %d', norm(us));

t = (ddt:ddt:length(es)*ddt);
figure
plot(t,es(:,1),'-r',t,es(:,2),'-g','LineWidth',1);
xlabel('Time (s)');
ylabel('Absolute error (m)');

figure
plot(t,xs(:,1),'-r',t,xs(:,2),'-g','LineWidth',1);
legend('X','Y');
xlabel('Time (s)');
ylabel('Displacement(m)');

