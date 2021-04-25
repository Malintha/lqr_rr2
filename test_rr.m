addpath('./mr/')

robot = get2RRobot();
rng(1);

start = [1 0];
target = [-0.5 0.5];
dt = 0.1;
t = (0:dt:5);

ik = inverseKinematics('RigidBodyTree', robot);
% get inverseK solution to stabilize around
A = [1 0; 0 1];
B = [1 0; 0 1]*dt;
L1 = 0.5;
L2 = 0.5;

% B = @(theta1,theta2) [-L1*sin(theta1)-L2*sin(theta1+theta2) -L2*sin(theta1+theta2); ...
%                     L1*cos(theta1)+L2*cos(theta1+theta2) L2*cos(theta1+theta2)];

R = [100 0; 0 100]; % penalize control
Q = [0.01 0; 0 0.01]; % penalize states
weights = [0, 0, 0, 1, 1, 0];
endEffector = 'tool';
qinit = homeConfiguration(robot);
y_tr = getTransform(robot, qinit, 'tool', 'base');
y = tform2trvec(y_tr);
y = y(1:2)';
qs = [];
k = lqr(A, B, Q, R);
st_tr = trvec2tform([start 0]);
target_tr = trvec2tform([target 0]);
x = y;

xs = [];
traj = CartesianTrajectory(st_tr, target_tr, 5, 5/dt, 3);

for i=1:length(traj)
    [~, x_bar] = TransToRp(traj{i});
    x_bar = x_bar(1:2);
    x_hat = (y - x_bar);
    ut = -k*x_hat(1:2);
    
    
%     do inverse kinematics and apply the control to kinematics robot
    xt = A*x + B*ut;  
    q = ik(endEffector, trvec2tform([xt' 0]), weights, qinit);
    
    xs = [xs xt];
    
    % adding a bit of observation noise
    y = xt + [randn(); randn()]/1000;
    fprintf('%d %d \n',xt,y);
    x = xt;
    
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
        hlines(j).MarkerSize=10; %changing marker size
    end

    drawnow
    filename = 'added_noise.gif';
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

function[robot] = get2RRobot()
    
    robot = rigidBodyTree('DataFormat','column','MaxNumBodies',3);
    L1 = 0.5;
    L2 = 0.5;

    body = rigidBody('link1');
    joint = rigidBodyJoint('joint1', 'revolute');
    setFixedTransform(joint,trvec2tform([0 0 0]));
    joint.JointAxis = [0 0 1];
    body.Joint = joint;
    addBody(robot, body, 'base');

    body = rigidBody('link2');
    joint = rigidBodyJoint('joint2','revolute');
    setFixedTransform(joint, trvec2tform([L1,0,0]));
    joint.JointAxis = [0 0 1];
    body.Joint = joint;
    addBody(robot, body, 'link1');

    body = rigidBody('tool');
    joint = rigidBodyJoint('fix1','fixed');
    setFixedTransform(joint, trvec2tform([L2, 0, 0]));
    body.Joint = joint;
    addBody(robot, body, 'link2');

    showdetails(robot)
end