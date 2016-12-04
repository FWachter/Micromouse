% PROGRAMMER: Frederick Wachter
% DATE CREATED: 26-04-2016
% PURPOSE: Creating an A* algorithm for 2d plane
% REFERENCE: http://ch.mathworks.com/matlabcentral/fileexchange/26248-a---a-star--search-for-path-planning-tutorial
% CONTACT INFO: wachterfreddy@gmail.com

% Please refer to the Wiki for instructions on how to use this script
% GITHUB WIKI: https://github.com/FWachter/Micromouse/wiki/MATLAB

%% Map Properties
map.MAX_X = 20;
map.MAX_Y = 20;
% map.coordinates = 2*ones(map.MAX_X,map.MAX_Y); % not needed since the map is being imported below


%% Map Definitions
map.legend.obstacle = -1;
map.legend.target = 0;
map.legend.robot = 1;
map.legend.space = 2;

axis([1,map.MAX_X+1,1,map.MAX_Y+1]);
grid on; hold on;


%% Get Map and Properties
cd(pwd);
map.coordinates = map.data;

[robot.xStartLocation,robot.yStartLocation] = find(map.coordinates==map.legend.robot);
[target.xLocation,target.yLocation] = find(map.coordinates==map.legend.target);
robot.location = [robot.xStartLocation,robot.yStartLocation];
target.location = [target.xLocation,target.yLocation];

for x = 1:map.MAX_X
    for y = 1:map.MAX_Y
        if (map.coordinates(x,y) == map.legend.obstacle)
            plot(x+.5,y+.5,'ro');
        elseif (map.coordinates(x,y) == map.legend.target)
            plot(x+.5,y+.5,'gd');
        elseif (map.coordinates(x,y) == map.legend.robot)
            plot(x+.5,y+.5,'bo');
        end
    end
end


%% A* ALGORITHM

% Variables
astar.open.list = []; 
astar.open.count = 0;
astar.closed.list = []; 
astar.closed.count = 0;
astar.path.isPath = 1;

% Functions
distance = @(location,target)(sqrt((target(1)-location(1))^2 + (target(2)-location(2))^2));
open = @(location,parent,cost,distance,pathLength)([1,location,parent,cost,distance,pathLength]);

% Pre-Processor
for x = 1:map.MAX_X
    for y = 1:map.MAX_Y
        if (map.coordinates(x,y) == map.legend.obstacle)
            astar.closed.count = astar.closed.count + 1;
            astar.closed.list(astar.closed.count,1:2) = [x,y];
        end
    end
end

astar.path.cost = 0;
astar.path.goalDistance = distance(robot.location,target.location);

astar.open.count = astar.open.count + 1;
astar.open.list(astar.open.count,:) = open(robot.location,robot.location,astar.path.cost,astar.path.goalDistance,astar.path.goalDistance);
astar.open.list(astar.open.count,1) = 0;

astar.closed.count = astar.closed.count + 1;
astar.closed.list(astar.closed.count,1:2) = robot.location;

fprintf('Starting Algorithm... '); tic;
% Algorithm to Search for Optimal Path
while (((robot.location(1) ~= target.location(1)) || (robot.location(2) ~= target.location(2))) && astar.path.isPath == 1)
    
    % Expand Array Function
    astar.expand.list = [];
    astar.expand.count = 0;
    for x = [1,0,-1]
        for y = [1,0,-1]
            if (((x == 0) && (y ~= 0)) || ((y == 0) && (x ~= 0)))
            % if ((x ~= y) || (x ~= 0))
                testPosition = [robot.location(1)+x,robot.location(2)+y];
                if (((testPosition(1) > 0) && (testPosition(1) <= map.MAX_X)) && ((testPosition(2) > 0) && (testPosition(2) <= map.MAX_Y)))
                    isOpen = 1; % set this location as a possible new location
                    for closedNode = 1:astar.closed.count
                        if (testPosition(1) == astar.closed.list(closedNode,1) && (testPosition(2) == astar.closed.list(closedNode,2)))
                            isOpen = 0; % this location has already been transversed
                        end
                    end
                    if (isOpen == 1)
                        astar.expand.count = astar.expand.count + 1;
                        astar.expand.list(astar.expand.count,1) = testPosition(1);
                        astar.expand.list(astar.expand.count,2) = testPosition(2);
                        astar.expand.list(astar.expand.count,3) = astar.path.cost + distance(robot.location,testPosition);
                        astar.expand.list(astar.expand.count,4) = distance(target.location,testPosition);
                        astar.expand.list(astar.expand.count,5) = astar.expand.list(astar.expand.count,3) + astar.expand.list(astar.expand.count,4);
                    end
                end
            end
        end
    end
    
    % Addition of Expanded Nodes Function
    for expandedNode = 1:astar.expand.count
        addNode = 0;
        for openNode = 1:astar.open.count
            if ((astar.expand.list(expandedNode,1) == astar.open.list(openNode,2)) && (astar.expand.list(expandedNode,2) == astar.open.list(openNode,3)))
                astar.open.list(openNode,8) = min(astar.open.list(openNode,8),astar.expand.list(expandedNode,5)); % check if the new path will be shorter than the original path
                if (astar.open.list(openNode,8) == astar.expand.list(expandedNode,5)) % if the new path is shorter
                    astar.open.list(openNode,4) = robot.location(1);
                    astar.open.list(openNode,5) = robot.location(2);
                    astar.open.list(openNode,6) = astar.expand.list(expandedNode,3);
                    astar.open.list(openNode,7) = astar.expand.list(expandedNode,4);
                end
                addNode = 1; % Node is to be added
            end
        end
        if (addNode == 0)
            astar.open.count = astar.open.count + 1;
            astar.open.list(astar.open.count,:) = open(astar.expand.list(expandedNode,1:2),robot.location,astar.expand.list(expandedNode,3),astar.expand.list(expandedNode,4),astar.expand.list(expandedNode,5));
        end
    end
    
    % Find Node With Minimum Cost
    astar.minimum.list = [];
    astar.minimum.count = 0;
    foundTarget = 0;
    for openNode = 1:astar.open.count
        if (astar.open.list(openNode,1) == 1)
            astar.minimum.count = astar.minimum.count + 1;
            astar.minimum.list(astar.minimum.count,:) = [astar.open.list(openNode,8),openNode];
            if ((astar.open.list(openNode,2) == target.location(1)) && (astar.open.list(openNode,3) == target.location(2)))
                foundTarget = 1;
                break;
            end
        end
    end
    if (foundTarget == 1) % if node is the target
        optimalNode = openNode;
    elseif (size(astar.minimum.list ~= 0)) % find smallest node in the open node list that is not a parent node
        [~,optimalNodeIndex] = min(astar.minimum.list(:,1));
        optimalNode = astar.minimum.list(optimalNodeIndex,2);
    else % if no more paths are available
        optimalNode = -1;
    end
    
    % Update Node for Next Iteration
    if (optimalNode ~= -1)
        robot.location = [astar.open.list(optimalNode,2),astar.open.list(optimalNode,3)];
        astar.path.cost = astar.open.list(optimalNode,6);
        astar.closed.count = astar.closed.count + 1;
        astar.closed.list(astar.closed.count,1:2) = [robot.location];
        astar.open.list(optimalNode,1) = 0;
    else
        astar.path.isPath = 0;
    end
    
end


%% Get Optimal Path with Generated Data
astar.optimal.list = [];
astar.optimal.count = 0;

finalX = astar.closed.list(end,1); % last x value from A* algorithm
finalY = astar.closed.list(end,2); % last y value from A* algorithm
astar.optimal.count = astar.optimal.count + 1;
astar.optimal.list(1,:) = [finalX,finalY];

if ((finalX == target.location(1)) && (finalY == target.location(2)))
    astar.solution.counter = 1;
    currentNode = find((astar.open.list(:,2)==finalX)&(astar.open.list(:,3)==finalY));
    parent = [astar.open.list(currentNode,4),astar.open.list(currentNode,5)];
    
    while ((parent(1) ~= robot.xStartLocation) || (parent(2) ~= robot.yStartLocation))
        astar.optimal.count = astar.optimal.count + 1;
        astar.optimal.list(astar.optimal.count,1:2) = parent;
        currentNode = find((astar.open.list(:,2)==parent(1))&(astar.open.list(:,3)==parent(2)));
        parent = [astar.open.list(currentNode,4),astar.open.list(currentNode,5)];
    end
    astar.optimal.count = astar.optimal.count + 1;
    astar.optimal.list(astar.optimal.count,1:2) = [robot.xStartLocation,robot.yStartLocation];
    time = toc;
    fprintf('Algorithm took %0.4f seconds to generate path\n',time);
    
    astar.solution.plotHandle = plot(astar.optimal.list(:,1)+.5,astar.optimal.list(:,2)+.5,'bo');
    astar.solution.plotHandle = plot(astar.optimal.list(:,1)+.5,astar.optimal.list(:,2)+.5,'b--');
else
    error('No Path Exists');
end



