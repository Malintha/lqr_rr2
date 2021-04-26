clear;
addpath('./mr/')

% L1, L2: arm lengths
L1 = 0.5;
L2 = 0.5;

robot = get2RRobot(L1, L2);
rng(1);

% start and goal positions for the end effector
start = [1 0];
target = [-0.5 0.8];
dt = 0.1;

% B = @(theta1,theta2) [-L1*sin(theta1)-L2*sin(theta1+theta2) -L2*sin(theta1+theta2); ...
%                     L1*cos(theta1)+L2*cos(theta1+theta2) L2*cos(theta1+theta2)];
%%
% y: observation
% x: state
% R: control penalization
% Q: state penalization
% A,B system matrices

A = [1 0; 0 1];
B = [1 0; 0 1]*dt;
R = [10 0; 0 10];
Q = [1 0; 0 1];

weights = [0, 0, 0, 1, 1, 0];
endEffector = 'tool';
qinit = homeConfiguration(robot);
y_tr = getTransform(robot, qinit, endEffector, 'base');
y = tform2trvec(y_tr);
y = y(1:2)';
qs = [];
k = lqr(A, B, Q, R);
st_tr = trvec2tform([start 0]);
target_tr = trvec2tform([target 0]);
x = y;

xs = [];
us = [];
es = [];
traj = CartesianTrajectory(st_tr, target_tr, 5, 5/dt, 5);

ik = inverseKinematics('RigidBodyTree', robot);
for i=1:length(traj) + 10
    if i<length(traj)
        [~, x_bar] = TransToRp(traj{i});
    else
        [~, x_bar] = TransToRp(traj{end});
    end
    x_bar = x_bar(1:2);
    x_hat = (y - x_bar);
    ut = -k*x_hat(1:2);
    
%     do inverse kinematics and apply the control to kinematics robot
    xt = A*x + B*ut;  
    q = ik(endEffector, trvec2tform([xt' 0]), weights, qinit);
    % calculate absolute error
    es = [es norm(xt-x_bar)];
    xs = [xs xt];
    us = [us ut];
    y = xt + x_hat*0.001; %observation
    x = xt; %state
    qinit = q;
    qs = [qs q];
end

figure
f1 = show(robot,qs(:,1));
view(2)
ax = gca;
ax.Projection = 'orthographic';
hold on
framesPerSecond = 1/dt;
r = rateControl(framesPerSecond);

for i = 1:length(qs)
    f1 = show(robot,qs(:,i),'PreservePlot',false);
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
    filename = 'added_noise1.gif';
    frame = getframe(1);
    im = frame2im(frame);
    [imind,cm] = rgb2ind(im,256);
    if i == 1 
      imwrite(imind,cm,filename,'gif', 'Loopcount',inf);
    else
      imwrite(imind,cm,filename,'gif','Writemode','append', 'DelayTime', dt);
    end
    waitfor(r);
end
hold off

%% draw the controls and state plots
fprintf('control: %d', norm(us));
t = (dt:dt:length(es)*dt);

figure
plot(t,es(1,:),'-r','LineWidth',2);
xlabel('Time (s)');
ylabel('Absolute error (m)');

figure
plot(t,xs(1,:),'-r',t,xs(2,:),'-g','LineWidth',2);
legend('X','Y');
xlabel('Time (s)');
ylabel('Displacement(m)');

