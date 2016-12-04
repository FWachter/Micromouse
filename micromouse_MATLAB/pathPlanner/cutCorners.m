%% ---- PROGRAM INFORMAITON ----
% PROGRAMMER: Frederick Wachter
% DATE CREATED: 2016-06-01
% PURPOSE: Cut corners for more realistic looking turns
% CONTACT INFO: wachterfreddy@gmail.com

% Please refer to the Wiki for instructions on how to use this script
% GITHUB WIKI: www.github.com/FWchter/Micromouse/Wiki

% If AStar has not been run yet
if ~(exist('astar','var'))
    filePath = mfilename('fullpath');
    cd(filePath(1:(find(filePath=='/',1,'last')-1)));
    load('../maps/20x20/orthogonal/map1.mat');
    astar = AStar_Structure_Fast;
    astar.runMap(map.data,1);
end

path = astar.robot.path;
newPath = path;

adjustment = 1-cosd(45);

tic;
previousPosition = path(end,:);
position = path(end-1,:);
if (previousPosition(1) == position(1))
    if (previousPosition(2) > position(2))
        previousDirection = 4;
    else
        previousDirection = 2;
    end
else
    if (previousPosition(1) > position(1))
        previousDirection = 1;
    else
        previousDirection = 3;
    end
end
previousPosition = position;

for index = (size(path,1)-2):-1:1
    position = path(index,:);
    if (previousPosition(1) == position(1))
        if (previousPosition(2) > position(2))
            if (previousDirection ~= 4)
                if (previousDirection == 1)
                    newPath(index+1,1) = newPath(index+1,1) + adjustment;
                    newPath(index+1,2) = newPath(index+1,2) - adjustment;
                else
                    newPath(index+1,1) = newPath(index+1,1) - adjustment;
                    newPath(index+1,2) = newPath(index+1,2) - adjustment;
                end
                previousDirection = 4;
            end
        else
            if (previousDirection ~= 2)
                if (previousDirection == 1)
                    newPath(index+1,1) = newPath(index+1,1) + adjustment;
                    newPath(index+1,2) = newPath(index+1,2) + adjustment;
                else
                    newPath(index+1,1) = newPath(index+1,1) - adjustment;
                    newPath(index+1,2) = newPath(index+1,2) + adjustment;
                end
                previousDirection = 2;
            end
        end
    else
        if (previousPosition(1) > position(1))
            if (previousDirection ~= 1)
                if (previousDirection == 2)
                    newPath(index+1,1) = newPath(index+1,1) - adjustment;
                    newPath(index+1,2) = newPath(index+1,2) - adjustment;
                else
                    newPath(index+1,1) = newPath(index+1,1) - adjustment;
                    newPath(index+1,2) = newPath(index+1,2) + adjustment;
                end
                previousDirection = 1;
            end
        else
            if (previousDirection ~= 3)
                if (previousDirection == 2)
                    newPath(index+1,1) = newPath(index+1,1) + adjustment;
                    newPath(index+1,2) = newPath(index+1,2) - adjustment;
                else
                    newPath(index+1,1) = newPath(index+1,1) + adjustment;
                    newPath(index+1,2) = newPath(index+1,2) + adjustment;
                end
                previousDirection = 3;
            end
        end
    end
    
    previousPosition = position;
end
time = toc;
fprintf('Corner cutting computation time: %.6f\n',time);

figure; plot(newPath(:,1),newPath(:,2));


