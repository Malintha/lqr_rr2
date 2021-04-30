% set the home configuration of the robot with a minimum distance to the
% ground plane from the end effector (d1)
% \alpha_1 is the bound for joint 1
% \alpha_2 is the bound for joint 2

% dy2 is the minimum height from the ground plane to the ef
% dy1 is the minimum height from the ground plane to the joint 2
% dy2 = L1sin(alpha1) + L2sin(alpha2)
% dy1 = L1sin(alpha1)

function[robot] = get2RRobot(L1, L2)
    
    robot = rigidBodyTree('DataFormat','column','MaxNumBodies',3);

    body = rigidBody('link1');
    joint = rigidBodyJoint('joint1', 'revolute');
    setFixedTransform(joint,trvec2tform([0 0 0]));
    joint.JointAxis = [0 0 1];
    body.Joint = joint;
    addBody(robot, body, 'base');

%     alpha1 = 0.2;
%     alpha2 = 0.2;
%     dy1 = L1*sin(alpha1);
%     dy2 = L1*sin(alpha1) + L2*sin(alpha2);
%     dx1 = L1*cos(alpha1);
%     dx2 = L1*cos(alpha1) + L2*cos(alpha2);
    
    
    body = rigidBody('link2');
    joint = rigidBodyJoint('joint2','revolute');
    setFixedTransform(joint, trvec2tform([L1,0,0]));
    joint.JointAxis = [0 0 1];
    body.Joint = joint;
    addBody(robot, body, 'link1');
%     r_12 = eul2rotm([0 0 1]);
%     tr_12 = [r_12 [dx2-dx1;dy2-dy1;0];[0 0 0 1]];
    
    body = rigidBody('tool');
    joint = rigidBodyJoint('fix1','fixed');

    setFixedTransform(joint, trvec2tform([L2, 0,0]));
    body.Joint = joint;
    addBody(robot, body, 'link2');

%     showdetails(robot)
end