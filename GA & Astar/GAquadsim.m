% � Rahul Kala, IIIT Allahabad, Creative Commons Attribution-ShareAlike 4.0 International License. 
% The use of this code, its parts and all the materials in the text; creation of derivatives and their publication; and sharing the code publically is permitted without permission. 
% Please cite the work in all materials as: R. Kala (2014) Code for Robot Path Planning using Genetic Algorithms, Indian Institute of Information Technology Allahabad, Available at: http://rkala.in/codes.html
clc
clear all;
close all;
% load examplemaps.mat;
% load map;
global map source goal splineSmoothing
%-------------------------------------------------------------------
% input map read from a bmp file. for new maps write the file name here
% map1 = robotics.BinaryOccupancyGrid(simplemap,3);
 
% map  = occupancyMatrix(map1)
% imshow(map);
I=imread('map2.bmp');
J=imresize(I,0.2);
map = double(imbinarize(J)); 

%==================================
%Define Number of Nodes
% xmax = 80;
% ymax = 40;
% %Start and Goal
%  source = [10, 20];
%  goal   = [490, 490];
source = [2, 2];
goal   = [15,15];

% %Nodes
% map = ones(xmax,ymax);
% %To define objects, set their map(x,y) to inf
% map(40,15:25) = 0;
% map(35:40,15) = 0;
% map(35:40,25) = 0;
surf(map')
colormap(gray)
view(2)

hold all
plot(source(1),source(2),'s','MarkerFaceColor','m');
plot(goal(1),goal(2),'s','MarkerFaceColor','m');

%-------------------------------------------------------------------
% no. of points that represent a candidate path, excluding 
% the source and goal. each point marks a robot turn.
noOfPointsInSolution = 3; 
NoOfGenerations = 30;
PopulationSize  = 100;
%-------------------------------------------------------------------
% use spline based smoothing. the code has a dependence 
% on the resoultion, and may be set to false if large changes 
% in resolution are made.
splineSmoothing = false; 
%-------------------------------------------------------------------
%%%%% parameters end here %%%%%
tic;
if ~feasiblePoint(source,map) 
    error('source lies on an obstacle or outside map'); end
if ~feasiblePoint(goal,map) 
    error('goal lies on an obstacle or outside map'); end
if noOfPointsInSolution<=0 
    error('noOfPointsInSolution should be greater than 1'); end
%-------------------------------------------------------------------
% currently the lower and upper bounds are taken as 0 and 1 respectively, 
% these will be re-scaled in phenotype gen. such that they lie inside map. 
% options=gaoptimset('PlotFcns', {@gaplotbestf, @gaplotdistance, ...
%     @gaplotrange},'Generations',NoOfGenerations,...
%     'PopulationSize',PopulationSize);
%-------------------------------------------------------------------
options = gaoptimset('PlotFcns', {@gaplotbestf, ...
    @gaplotrange},'Generations',NoOfGenerations,...
    'PopulationSize',PopulationSize);
%-------------------------------------------------------------------
[solution, cost] = ga(@PathCostGA, noOfPointsInSolution*2,[],[],[],[],...
zeros(noOfPointsInSolution*2,1),ones(noOfPointsInSolution*2,1),[],options);
%-------------------------------------------------------------------
% disp('click/press any key');
% waitforbuttonpress; 
%-------------------------------------------------------------------
if PathCostGA(solution)>size(map,1)*size(map,2) 
    % indicating an infeasible path due to large cost due to penalties
    error('no path found');
end
fprintf('processing time=%d \nPath Length=%d \n\n', toc,cost);
%-------------------------------------------------------------------
path=[source; [solution(1:2:end)'*size(map,1) ...
      solution(2:2:end)'*size(map,2)]; goal]; 
% souce and goal is fixed. other points are from the GA individual repr.
%-------------------------------------------------------------------
if splineSmoothing
    path = bsp(path); 
    % a point based specification of path is smoothed by using splines
end
%-------------------------------------------------------------------
map1 = map;
map1(map1==0) = inf;
map1(map1==1) = 0;
close all;
surf(map1');
colormap(gray);
view(2);
hold all
plot(source(1),source(2),'s','MarkerFaceColor','m');
plot(goal(1),goal(2),'s','MarkerFaceColor','m');

for j = 1:size(path,1)
    plot(path(j,1),path(j,2),'x','color','r')
%pause(0.001)
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
waypoints = path;
%Clearing the workspace.
%Getting the waypoints from path

%Defining the Start and Endpoints
%startpoint = waypoints(1,:);
endpoint = waypoints(end,:);

%Establishing the VM and Host IP addresses
VMIP='192.168.9.131';
HostIP='192.168.1.87';

%Initializing the master node and connecting the host and vm together.
try
    rosinit(VMIP, 'NodeHost', HostIP);
catch
    disp('Error: Unable to initialize the master node. Terminating script...')
    quit;
end
%testing to see if the connection is active
try
    rostopic list;
catch
    disp('Error: Unable to connect to ROS network. Terminating script...')
    quit;
end

%Creating a cmd_vel publisher and several cmd_vel messages
velPub = rospublisher('cmd_vel');
velMsg = rosmessage(velPub);

hover = rosmessage(velPub);
hover.Linear.Z = 1.0;

stop = rosmessage(velPub);
stop.Linear.X = 0.0;
stop.Linear.Y = 0.0;
stop.Linear.Z = 0.0;
stop.Angular.Z = 0.0;

%UAV Hover;
send(velPub, hover);
pause(5);
send(velPub, stop);
pause(2);
%Setting up a odometry subscriber and getting initial position and
%orientation
odom = rossubscriber('/gazebo/model_states');
uavPose = getPose(odom);

%Defining the UAV's controller and parameters
controller = robotics.PurePursuit;
controller.Waypoints = waypoints;
controller.DesiredLinearVelocity = 0.3;
controller.MaxAngularVelocity = 2.0;
controller.LookaheadDistance = 0.5;

%Defining the minToGoal and the distanceToGoal and controllRate
minToGoal = 0.75;
distanceToGoal = norm(uavPose(1:2) - endpoint);
while(distanceToGoal > minToGoal)
    
    %calculating linear and angular velocity of uav from controller object
    [velocity, angularVelocity] = controller(uavPose);
    
    %creating new velocity message for UAV
    velMsg.Linear.X = velocity;
    velMsg.Angular.Z = angularVelocity;
    
    %sending message to UAV
    send(velPub, velMsg);
    pause(1);
    
    %sending stop to get new pose data
    send(velPub, stop);
    
    %getting new pose
    uavPose = getPose(odom);
    
    %recalculating the distance to endpoint
    distanceToGoal = norm(uavPose(1:2) - endpoint);
end

send(velPub, stop);

rosshutdown;


function uavPose = getPose(odom)
    odomdata = receive(odom,3);
    pose = odomdata.Pose(2);
    quat = pose.Orientation;
    angles = quat2eul([quat.W quat.X quat.Y quat.Z]);
    uavPose = [pose.Position.X, pose.Position.Y, angles(1)];    
end