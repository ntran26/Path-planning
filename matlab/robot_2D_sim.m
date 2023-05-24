clear all;

% define a set of waypoints for the desired path for the robot
path = [2.00    1.00;
        1.25    1.75;
        5.25    8.25;
        7.25    8.75;
        11.75   10.75;
        12.00   10.00];

% set the current location and the goal location of the robot as defined by the path
robotInitialLocation = path(1,:);
robotGoal = path(end,:);

% assume an initial robot orientation
initialOrientation = 0;

% define the current pose for the robot [x y theta]
robotCurrentPose = [robotInitialLocation initialOrientation]';

% create a robot kinematic model
robot = differentialDriveKinematics("TrackWidth", 1, "VehicleInputs", "VehicleSpeedHeadingRate");

% visualize the path
figure
plot(path(:,1), path(:,2),'k--d')
% xlim([0 13])
% ylim([0 13])

% define path following controller
controller = controllerPurePursuit;

% use 'path' to set the desired waypoint
controller.Waypoints = path;

% set controller parameters
controller.DesiredLinearVelocity = 0.6;
controller.MaxAngularVelocity = 2;
controller.LookaheadDistance = 0.3;

goalRadius = 0.1;
distanceToGoal = norm(robotInitialLocation - robotGoal);

% initialize the simulation loop
sampleTime = 0.1;
vizRate = rateControl(1/sampleTime);

figure
show()
frameSize = robot.TrackWidth/0.8;

while( distanceToGoal > goalRadius )
    
    % Compute the controller outputs (inputs to the robot)
    [v, omega] = controller(robotCurrentPose);
    
    % Get the robot's velocity using controller inputs
    vel = derivative(robot, robotCurrentPose, [v omega]);
    
    % Update the current pose
    robotCurrentPose = robotCurrentPose + vel*sampleTime; 
    
    % Re-compute the distance to the goal
    distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal(:));
    
    % Update the plot
    hold off
    
    % Plot path each instance so that it stays persistent while robot mesh
    % moves
    plot(path(:,1), path(:,2),"k--d")
    hold all
    
    % Plot the path of the robot as a set of transforms
    plotTrVec = [robotCurrentPose(1:2); 0];
    plotRot = axang2quat([0 0 1 robotCurrentPose(3)]);
    plotTransforms(plotTrVec', plotRot, "MeshFilePath", "groundvehicle.stl", "Parent", gca, "View","2D", "FrameSize", frameSize);
    light;
    xlim([0 13])
    ylim([0 13])
    
    waitfor(vizRate);
end








