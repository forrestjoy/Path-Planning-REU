%Anthony Chrabieh
%Bonus Problem
%A* Algorithm

clear all
clc
clf
i=imread('map2.bmp');
J=imresize(i,0.2);
map=imbinarize(J);
% source=[5 5];
% goal=[45 45];
% map=im2bw(imread('map4.bmp')); % input map read from a bmp file. for new maps write the file name here
source=[5 5]; % source position in Y, X format
goal=[95 95]; % goal position in Y, X format
% %Define Number of Nodes
%  xmax = 80;
%  ymax = 40;

% %source and Goal
% source = [10, 20];
% goal   = [79, 20];
% 
% % source = [10,ymax/2];
% % goal = [xmax,ymax/2];
% 
% %Nodes
% map = zeros(xmax,ymax);
% % 
% %To define objects, set their map(x,y) to inf
%  map(40,15:25) = inf;
%  map(35:40,15) = inf;
%  map(35:40,25) = inf;
%  map(35:40,19) = inf;
%  map(35:40,21) = inf;
%  map(35,7:15)  = inf;
%  map(25:35,7)  = inf;
% map(35,) = inf;

%To use Random Objects, uncomment this secion
% NumberOfObjects = 50;
% k = 0;
% while (k < NumberOfObjects)
%     length = max(3,randi(20));
%     x = randi(xmax-2*length)+length;
%     y = randi(ymax-2*length)+length;
%     direction = randi(4);
%     if (direction == 1)
%         x = x:x+length;
%     elseif (direction == 2)
%         x = x-length:x;
%     elseif (direction == 3)
%         y = y:y+length;
%     elseif (direction == 4)
%         y = y-length:y;
%     end
%     if (sum(isinf(map(x,y)))>0)
%         continue
%     end
%     map(x,y) = inf;
%     k = k + 1;
% end


%Heuristic Weight%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%weight = sqrt(2); %Try 1, 1.5, 2, 2.5
%Increasing weight makes the algorithm greedier, and likely to take a
%longer path, but with less computations.
weight = 0; %gives Djikstra algorithm
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


tic;
%Heuristic map of all nodes
for x = 1:size(map,1)
    for y = 1:size(map,2)
        if(map(x,y)~=1)
            H(x,y) = weight*norm(goal-[x,y]);
            G(x,y) = 1;
        else
            H(x,y)=-1;
            G(x,y)=inf;
        end
    end
end

%Plotting%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
map1 = double(map);
imshow(map1);
map1(map1==0) = inf;
map1(map1==1) = 0;
close all;
%%
surf(map1');
colormap(gray);
view(2);

hold all
plot(source(1),source(2),'s','MarkerFaceColor','b')
plot(goal(1),goal(2),'s','MarkerFaceColor','m')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%initial conditions%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
G(source(1),source(2)) = 0;
F(source(1),source(2)) = H(source(1),source(2));

closedNodes = [];
openNodes = [source G(source(1),source(2)) F(source(1),source(2)) 0]; %[x y G F cameFrom]
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Solve
solved = false;
%imshow(map);
% view(2);
while(~isempty(openNodes))
    
    %pause(0.000001)
    
    %find node from open set with smallest F value
    [A,I] = min(openNodes(:,4));
    
    %set current node
    current = openNodes(I,:);
    plot(current(1),current(2),'o','color','g','MarkerFaceColor','g')
    
    %if goal is reached, break the loop
    if(current(1:2)==goal)
        closedNodes = [closedNodes;current];
        solved = true;
        break;
    end
    
    %remove current node from open set and add it to closed set
    openNodes(I,:) = [];
    closedNodes = [closedNodes;current];
    
    %for all neighbors of current node
    for x = current(1)-1:current(1)+1
        for y = current(2)-1:current(2)+1
            
            %if out of range skip
%             if (x<1||x>xmax||y<1||y>ymax)
%                 continue
%             end
            if (x<1||x>length(map)||y<1||y>length(map))
                continue
            end
            
            %if object skip
            if (isinf(map1(x,y)))
                continue
            end
%             x,y
%             map(x,y)
            %if current node skip
            if (x==current(1)&&y==current(2))
                continue
            end
            
            %if already in closed set skip
            skip = 0;
            for j = 1:size(closedNodes,1)
                if(x == closedNodes(j,1) && y==closedNodes(j,2))
                    skip = 1;
                    break;
                end
            end
            if(skip == 1)
                continue
            end
            
            A = [];
            %Check if already in open set
            if(~isempty(openNodes))
                for j = 1:size(openNodes,1)
                    if(x == openNodes(j,1) && y==openNodes(j,2))
                        A = j;
                        break;
                    end
                end
            end
            
            newG = G(current(1),current(2)) + round(norm([current(1)-x,current(2)-y]),1);
            
            %if not in open set, add to open set
            if(isempty(A))
                G(x,y) = newG;
                newF = G(x,y) + H(x,y);
                newNode = [x y G(x,y) newF size(closedNodes,1)];
                openNodes = [openNodes; newNode];
                plot(x,y,'x','color','b')
                continue
            end
            
            %if no better path, skip
            if (newG >= G(x,y))
                continue
            end
            
            G(x,y) = newG;
            newF = newG + H(x,y);
            openNodes(A,3:5) = [newG newF size(closedNodes,1)];
        end
    end
end
if (solved)
    %Path plotting
    j = size(closedNodes,1);
    path = [];
    while(j > 0)
        x = closedNodes(j,1);
        y = closedNodes(j,2);
        j = closedNodes(j,5);
        path = [x,y;path];
    end
    for j = 1:size(path,1)
        plot(path(j,1),path(j,2),'x','color','r')
        %pause(0.01)
    end
else
    disp('No Path Found')
end
length(path)
toc
%path