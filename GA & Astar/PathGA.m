% © Rahul Kala, IIIT Allahabad, Creative Commons Attribution-ShareAlike 4.0 International License. 
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
J=imresize(I,0.1)
map = double(imbinarize(J)); 

%==================================
%Define Number of Nodes
% xmax = 80;
% ymax = 40;
% %Start and Goal
%  source = [10, 20];
%  goal   = [490, 490];
source = [5, 5];
goal   = [45,45];

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
NoOfGenerations = 20;
PopulationSize  = 200;
%-------------------------------------------------------------------
% use spline based smoothing. the code has a dependence 
% on the resoultion, and may be set to false if large changes 
% in resolution are made.
splineSmoothing = true; 
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
surf(map1')
colormap(gray)
view(2)
hold all
plot(source(1),source(2),'s','MarkerFaceColor','m');
plot(goal(1),goal(2),'s','MarkerFaceColor','m');

for j = 1:size(path,1)
    plot(path(j,1),path(j,2),'x','color','r')
%pause(0.001)
end

