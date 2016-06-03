%% ---- PROGRAM INFORMAITON ----
% PROGRAMMER: Frederick Wachter
% DATE CREATED: 2016-06-01
% PURPOSE: Interpolate turns for more realistic paths
% CONTACT INFO: wachterfreddy@gmail.com

% Please refer to the Wiki for instructions on how to use this script
% GITHUB WIKI: www.github.com/FWchter/Micromouse/Wiki

% -------------------------------
% Need to fix case for lane shift
% -------------------------------

% If AStar has not been run yet
if ~(exist('astar','var'))
    filePath = mfilename('fullpath');
    cd(filePath(1:(find(filePath=='/',1,'last')-1)));
    load('../maps/20x20/orthogonal/map1.mat');
    astar = AStar_Structure_Fast;
    astar.runMap(map.data,1);
end

path = astar.robot.path;
newPath = path(end,:);

adjustment = 1-cosd(45);

topRight_21    = [cosd(10:10:80)',sind(10:10:80)'];
topRight_34    = [cosd(80:-10:10)',sind(80:-10:10)'];
topLeft_14     = [cosd(100:10:170)',sind(100:10:170)'];
topLeft_23     = [cosd(170:-10:100)',sind(170:-10:100)'];
bottomLeft_43  = [cosd(190:10:260)',sind(190:10:260)'];
bottomLeft_12  = [cosd(260:-10:190)',sind(260:-10:190)'];
bottomRight_32 = [cosd(280:10:350)',sind(280:10:350)'];
bottomRight_41 = [cosd(350:-10:280)',sind(350:-10:280)'];

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
newPath = [newPath;previousPosition];
previousPosition = position;

% figure; h = plot(newPath(:,1),newPath(:,2));
% axis([1,21,1,21]);
for index = (size(path,1)-2):-1:1
    position = path(index,:);
    if (previousPosition(1) == position(1))
        if (previousPosition(2) > position(2))
            if (previousDirection ~= 4)
                if (previousDirection == 1)
                    newPath = [newPath;[topLeft_14(:,1)+1+position(1),topLeft_14(:,2)+position(2)]];
                else
                    newPath = [newPath;[topRight_34(:,1)-1+position(1),topRight_34(:,2)+position(2)]];
                end
                previousDirection = 4;
            else
                newPath = [newPath;previousPosition];
            end
        else
            if (previousDirection ~= 2)
                if (previousDirection == 1)
                    newPath = [newPath;[bottomLeft_12(:,1)+1+position(1),bottomLeft_12(:,2)+position(2)]];
                else
                    newPath = [newPath;[bottomRight_32(:,1)-1+position(1),bottomRight_32(:,2)+position(2)]];
                end
                previousDirection = 2;
            else
                newPath = [newPath;previousPosition];
            end
        end
    else
        if (previousPosition(1) > position(1))
            if (previousDirection ~= 1)
                if (previousDirection == 2)
                    newPath = [newPath;[topRight_21(:,1)+position(1),topRight_21(:,2)-1+position(2)]];
                else
                    newPath = [newPath;[bottomRight_41(:,1)+position(1),bottomRight_41(:,2)+1+position(2)]];
                end
                previousDirection = 1;
            else
                newPath = [newPath;previousPosition];
            end
        else
            if (previousDirection ~= 3)
                if (previousDirection == 2)
                    newPath = [newPath;[topLeft_23(:,1)+position(1),topLeft_23(:,2)-1+position(2)]];
                else
                    newPath = [newPath;[bottomLeft_43(:,1)+position(1),bottomLeft_43(:,2)+1+position(2)]];
                end
                previousDirection = 3;
            else
                newPath = [newPath;previousPosition];
            end
        end
    end
%     set(h,'XData',newPath(:,1));
%     set(h,'YData',newPath(:,2));
%     drawnow;
    
    previousPosition = position;
end
time = toc;
fprintf('Corner cutting computation time: %.6f\n',time);

figure; plot(newPath(:,1),newPath(:,2));




