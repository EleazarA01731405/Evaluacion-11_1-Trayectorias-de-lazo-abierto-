%% EXAMPLE: Differential drive vehicle following waypoints using the 
% Pure Pursuit algorithm
%
% Copyright 2018-2019 The MathWorks, Inc.

%% Define Vehicle
R = 0.1;                % Wheel radius [m]
L = 0.5;                % Wheelbase [m]
dd = DifferentialDrive(R,L);

%% Simulation parameters
sampleTime = 0.01;               % Sample time [s]
tVec = 0:sampleTime:77;         % Time array
    
initPose = [0.5;3;180];             % Initial pose (x y theta)
pose = zeros(3,numel(tVec));    % Pose matrix
pose(:,1) = initPose;

% Define waypoints
waypoints = [0.5,3;
             0,3;
             0,1.5;
             0.5,1.5;
             0,1.5;
             0,0;
             1,0;%Final E
             1,3;
             1,0;
             2,0;%Final L
             2,3;
             2.5,3;
             2,3;
             2,1.5;
             2.5,1.5;
             2,1.5;
             2,0;%Final R
             3,0;
             3,3;
             3.5,3;
             3.5,1.5;
             3,1.5;
             3.5,1.5;
             3.5,0;%Final de la A
             4,0;
             4.5,3;
             4,3;
             4.5,3;
             4,0;%Final de la Z
             5,0;
             5,3;
             5.5,3;
             5.5,1.5;
             5,1.5;
             5.5,1.5;
             5.5,0;%Final A
             6,0;
             6,3;
             6.5,3;
             6.5,1.5;
             6,1.5;
             6.5,0;%Final R
             8.5,0;%Final _
             8.5,3;
             8,3;
             8,0;%Final O
             9,0;
             9,3;
             9.5,3;
             9,3;
             9,0;
             9.5,0;
             9.5,1.5;
             9.25,1.5];%Final de la G

% Create visualizer
viz = Visualizer2D;
viz.hasWaypoints = true;

%% Pure Pursuit Controller
controller = controllerPurePursuit;
controller.Waypoints = waypoints;
controller.LookaheadDistance = .1;
controller.DesiredLinearVelocity = 1;
controller.MaxAngularVelocity = 10;

%% Simulation loop
close all
r = rateControl(1/sampleTime);
for idx = 2:numel(tVec) 
    % Run the Pure Pursuit controller and convert output to wheel speeds
    [vRef,wRef] = controller(pose(:,idx-1));
    [wL,wR] = inverseKinematics(dd,vRef,wRef);
 
    
    % Compute the velocities
    [v,w] = forwardKinematics(dd,wL,wR);
    velB = [v;0;w]; % Body velocities [vx;vy;w]
    vel = bodyToWorld(velB,pose(:,idx-1));  % Convert from body to world
    
    % Perform forward discrete integration step
    pose(:,idx) = pose(:,idx-1) + vel*sampleTime; 
    
    % Update visualization
    viz(pose(:,idx),waypoints)
    waitfor(r);
    
end