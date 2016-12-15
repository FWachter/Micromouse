%% ---- PROGRAM INFORMAITON ----
% PROGRAMMER: Frederick Wachter
% DATE CREATED: 2016-06-01
% PURPOSE: Generate direction field for solution path
% CONTACT INFO: wachterfreddy@gmail.com

% Please refer to the Wiki for instructions on how to use this script
% GITHUB WIKI: https://github.com/FWachter/Micromouse/wiki/MATLAB

% If AStar has not been run yet
if ~(exist('astar','var'))
    filePath = mfilename('fullpath');
    cd(filePath(1:(find(filePath=='/',1,'last')-1)));
    load('../maps/20x20/orthogonal/map1.mat');
    astar = AStar_Structure_Fast;
    astar.runMap(map.data,1);
end

path = astar.robot.path;
map = astar.map.coordinates;

directionField = zeros(size(map,1),size(map,2));

tic;
for index = (size(path,1)-1):-1:1
    previousPosition = path(index+1,:);
    position = path(index,:);
    
    if (previousPosition(1) == position(1))
        if (previousPosition(2) > position(2))
            directionField(previousPosition(1),previousPosition(2)) = 4;
        else
            directionField(previousPosition(1),previousPosition(2)) = 2;
        end
    else
        if (previousPosition(1) > position(1))
            directionField(previousPosition(1),previousPosition(2)) = 1;
        else
            directionField(previousPosition(1),previousPosition(2)) = 3;
        end
    end
end
time = toc;

fprintf('Direction field computation time: %.6f\n',time);
figure; barHandle = bar3(directionField);
xlabel('X Position'); ylabel('Y Position'); zlabel('Movement Direction');
for k = 1:length(barHandle) % adjust color scheme to height
    zdata = barHandle(k).ZData;
    barHandle(k).CData = zdata;
    barHandle(k).FaceColor = 'interp';
end


