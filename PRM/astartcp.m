%© Rahul Kala, IIIT Allahabad, Creative Commons Attribution-ShareAlike 4.0 International License. 
% The use of this code, its parts and all the materials in the text; creation of derivatives and their publication; and sharing the code publically is permitted without permission. 
% Please cite the work in all materials as: R. Kala (2014) Code for Robot Path Planning using Probabilistic Roadmap, Indian Institute of Information Technology Allahabad, Available at: http://rkala.in/codes.html

i=imread('map2.bmp');
J=imresize(i,0.2);
map=double(imbinarize(J));
source=[5 5];
goal=[95 95];
% map=im2bw(imread('map2.bmp')); % input map read from a bmp file. for new maps write the file name here
% source=[10 10]; % source position in Y, X format
% goal=[490 490]; % goal position in Y, X format
k=50; % number of points in the PRM
display=true; % display processing of nodes

%%%%% parameters end here %%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
map1 = map;
% imshow(map1);
map1(map1==0) = inf;
map1(map1==1) = 0;
close all;
surf(map1');
colormap(gray);
view(2);

hold all
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if ~feasiblePoint2(source,map1'), error('source lies on an obstacle or outside map'); end
if ~feasiblePoint2(goal,map1'), error('goal lies on an obstacle or outside map'); end
%%
%imshow(map);
%plot('position',[1 1 size(map)-1],'edgecolor','k')
vertex=[source;goal]; % source and goal taken as additional vertices in the path planning to ease planning of the robot
if display, plot(vertex(1,2),vertex(1,1),'o','color','g'); end
if display, plot(vertex(2,2),vertex(2,1),'o','color','g'); end
tic;
while length(vertex)<k+2 % iteratively add vertices
    x=double(int32(rand(1,2) .* size(map1')));
    if feasiblePoint2(x,map1') 
        vertex=[vertex;x]; 
        if display, plot(x(2),x(1),'x','color','r'); end
    end
end
% if display 
%     disp('click/press any key');
%     waitforbuttonpress; 
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
edges=cell(k+2,1); % edges to be stored as an adjacency list
for i=1:k+2
    for j=i+1:k+2
        u=zeros([k+2 2]);
        if checkPath(vertex(i,:),vertex(j,:),map1');           
            edges{i}=[edges{i};j];edges{j}=[edges{j};i];
             if display, plot3([vertex(i,2);vertex(j,2)],[vertex(i,1);vertex(j,1)],u);%pause(0.000001); 
             end
        end
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% if display 
%     disp('click/press any key');
%     waitforbuttonpress; 
% end
%structure of a node is taken as index of node in vertex, historic cost, heuristic cost, total cost, parent index in closed list (-1 for source) 
Q=[1 0 heuristic(vertex(1,:),goal) 0+heuristic(vertex(1,:),goal) -1]; % the processing queue of A* algorihtm, open list
closed=[]; % the closed list taken as a list
pathFound=false;
while size(Q,1)>0
     [A, I]=min(Q,[],1);
     n=Q(I(4),:); % smallest cost element to process
     Q=[Q(1:I(4)-1,:);Q(I(4)+1:end,:)]; % delete element under processing
     if n(1)==2 % goal test
         pathFound=true;break;
     end
     for mv=1:length(edges{n(1),1}) %iterate through all edges from the node
         newVertex=edges{n(1),1}(mv);
         if length(closed)==0 || length(find(closed(:,1)==newVertex))==0 % not already in closed
             historicCost=n(2)+historic(vertex(n(1),:),vertex(newVertex,:));
             heuristicCost=heuristic(vertex(newVertex,:),goal);
             totalCost=historicCost+heuristicCost;
             add=true; % not already in queue with better cost
             if length(find(Q(:,1)==newVertex))>=1
                 I=find(Q(:,1)==newVertex);
                 if Q(I,4)<totalCost, add=false;
                 else Q=[Q(1:I-1,:);Q(I+1:end,:);];add=true;
                 end
             end
             if add
                 Q=[Q;newVertex historicCost heuristicCost totalCost size(closed,1)+1]; % add new nodes in queue
             end
         end           
     end
     closed=[closed;n]; % update closed lists
end
if ~pathFound
    error('no path found')
end

fprintf('processing time=%d \nPath Length=%d \n\n', toc,n(4)); 
path=[vertex(n(1),:)]; %retrieve path from parent information

prev=n(5);
while prev>0
    path=[vertex(closed(prev,1),:);path];
    prev=closed(prev,5);
end
z=zeros([length(path) 1]);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% map1 = double(map);
% % imshow(map1);
% map1(map1==0) = inf;
% map1(map1==1) = 0;
close all;
surf(map1');
colormap(gray);
view(2);

hold all
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% imshow(map);
%plot3([1 1 size(map)-1],'color','k')
plot3(path(:,2),path(:,1),z,'color','r');