clear all;

% import image and transform to binary
img = imread('map.jpg');
%img = imread('maze.png');
img = rgb2gray(img);
img = img < 100;
img = imresize(img, 0.1);

% create binary occupancy map from the binary image
img = occupancyMap(img,1);

% get map dimensions
map = getOccupancy(img);
[row, col] = size(map);

% create borders (ones(rows, columns))
border = 1;
setOccupancy(img, [0 0], ones(row, border))  % left border 
setOccupancy(img, [col-border 0], ones(row, border))  % right border
setOccupancy(img, [0 row-border], ones(border, col))  % top border
setOccupancy(img, [0 0], ones(border, col))   % bottom border

% inflate map to avoid going too close to the obstacles
map = copy(img);
inflate(map,4);

% show original and inflated map
figure
subplot(2,1,1)
show(img)
subplot(2,1,2)
show(map)

% select start and goal points
figure
show(map)
disp('Select start point');
[x1,y1] = ginput(1);
disp('Select goal point');
[x2,y2] = ginput(1);

x1 = round(x1);
y1 = round(y1);
x2 = round(x2);
y2 = round(y2);

start = [y1 x1]
goal = [y2 x2]

% figure
% show(map)

path = Astar(map, start, goal);

figure
show(img)
hold on;
grid on;
plot(path(:,2),path(:,1),'r','LineWidth',2)

% flip the x and y axis of the path coordinates
p = path(:,1);
path(:,1) = path(:,2);
path(:,2) = p;

% set the current location and the goal location of the robot as defined by the path
robotInitialLocation = path(1,:);
robotGoal = path(end,:);

% assume an initial robot orientation
initialOrientation = 0;

% define the current pose for the robot [x y theta]
robotCurrentPose = [robotInitialLocation initialOrientation]';

% create a robot kinematic model
robot = differentialDriveKinematics("TrackWidth", 1, "VehicleInputs", "VehicleSpeedHeadingRate");

% define path following controller
controller = controllerPurePursuit;

% use 'path' to set the desired waypoint
controller.Waypoints = path;

% set controller parameters
controller.DesiredLinearVelocity = 0.6;
controller.MaxAngularVelocity = 2;
controller.LookaheadDistance = 2;

% controller.DesiredLinearVelocity = 0.6;
% controller.MaxAngularVelocity = 2;
% controller.LookaheadDistance = 0.3;

goalRadius = 0.1;
distanceToGoal = norm(robotInitialLocation - robotGoal);

% Initialize the simulation loop
sampleTime = 0.2;
vizRate = rateControl(1/sampleTime);

reset(vizRate);

% Initialize the figure
figure
%frameSize = robot.TrackWidth/0.8;
frameSize = 5*robot.TrackWidth;

while( distanceToGoal > goalRadius )
    
    % Compute the controller outputs, i.e., the inputs to the robot
    [v, omega] = controller(robotCurrentPose);
    
    % Get the robot's velocity using controller inputs
    vel = derivative(robot, robotCurrentPose, [v omega]);
    
    % Update the current pose
    robotCurrentPose = robotCurrentPose + vel*sampleTime; 
    
    % Re-compute the distance to the goal
    distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal(:));
    
    % Update the plot
    hold off
    show(img);
    hold all

    % Plot path each instance so that it stays persistent while robot move
    plot(path(:,1), path(:,2), '--')
    scatter(path(1,1), path(1,2), 20, 'r', 'filled')
    scatter(path(end,1), path(end,2), 20, 'b', 'filled')
    
    % Plot the path of the robot as a set of transforms
    plotTrVec = [robotCurrentPose(1:2); 0];
    plotRot = axang2quat([0 0 1 robotCurrentPose(3)]);
    plotTransforms(plotTrVec', plotRot, 'MeshFilePath', 'groundvehicle.stl', 'Parent', gca, "View","2D", "FrameSize", frameSize);
    light;

    waitfor(vizRate);
end