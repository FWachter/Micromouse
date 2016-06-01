%% ---- PROGRAM INFORMAITON ----
% PROGRAMMER: Frederick Wachter
% DATE CREATED: 2016-06-01
% PURPOSE: Micromouse map generator
% REFERENCE: http://ch.mathworks.com/matlabcentral/fileexchange/26248-a---a-star--search-for-path-planning-tutorial
% CONTACT INFO: wachterfreddy@gmail.com

% Please refer to the Wiki for instructions on how to use this script
% GITHUB WIKI: www.github.com/FWchter/Micromouse/Wiki


classdef mapGenerator < handle
    
    properties(SetAccess = protected)
        limits
        legend
        data
        inputs
        window
    end
    

%% CONSTRUCTOR METHOD
    methods
        
        function map = mapGenerator(varargin)
        % EXAMPLE FUNCTION CALL: map = mapGenerator([20,20]);
        % PROGRAMMER: Frederick Wachter
        % DATE CREATED: 2016-06-01
        % PURPOSE: Constructs the structure
            
            % Default Map Legend
            map.legend.obstacle  = -1;
            map.legend.target    = 0;
            map.legend.start     = 1;
            map.legend.freeSpace = 2;
            
            % Initialize Counters
            map.inputs.obstacles      = 0;
            map.inputs.targetLocation = zeros(0,0);
            map.inputs.startLocation  = zeros(0,0);
            
        end
        
    end

    
%% PUBLIC METHODS
    methods
        
        function generateMap(map) 
        % EXAMPLE FUNCTION CALL: setupFigure(map);
        % PROGRAMMER: Frederick Wachter
        % DATE CREATED: 2016-06-01
        % PURPOSE: Calls all relevent functions to create and save a map
        
            map.getMapDimensions();
            map.setupFigure();
            map.selectTarget();
            map.selectObstacles();
            map.selectStart();
            map.exportData();
        
        end
        
        function getMapDimensions(map)
        % EXAMPLE FUNCTION CALL: setupFigure(map);
        % PROGRAMMER: Frederick Wachter
        % DATE CREATED: 2016-06-01
        % PURPOSE: Set the limits of the map
            
            map.limits.MAX_X = input('Size of Map (X): '); % get max x value for map
            map.limits.MAX_Y = input('Size of Map (Y): '); % get max y value for map
            
            map.data = 2*ones(map.limits.MAX_X,map.limits.MAX_Y); % initialize map
            
        end
        
        function setMapLegend(map)
        % EXAMPLE FUNCTION CALL: setupFigure(map);
        % PROGRAMMER: Frederick Wachter
        % DATE CREATED: 2016-06-01
        % PURPOSE: Reset the map legend away from default value
            
            map.legend.obstacle  = input('Obstacle Location Legend: '); % set the obstacle location legend value
            map.legend.target    = input('Target Location Legend: '); % set the target location legend value
            map.legend.start     = input('Start Location Legend: '); % set the start location legend value
            map.legend.freeSpace = input('Free Space Legend: '); % set the free space location legend value
            
            map.data = map.legend.freeSpace*ones(map.limits.MAX_X,map.limits.MAX_Y); % reset the map to desired legend
            
        end
       
        function setupFigure(map)
        % EXAMPLE FUNCTION CALL: setupFigure(map);
        % PROGRAMMER: Frederick Wachter
        % DATE CREATED: 2016-06-01
        % PURPOSE: Initialized the figure
            
            map.window.handle = figure('Name','Map Generator','NumberTitle','off');
            axis([1,map.limits.MAX_X+1,1,map.limits.MAX_Y+1]);
            grid on; hold on;
            
            map.window.leftMouseButton = 1;
            
        end
        
        function selectTarget(map)
        % EXAMPLE FUNCTION CALL: selectTarget(map);
        % PROGRAMMER: Frederick Wachter
        % DATE CREATED: 2016-06-01
        % PURPOSE: Get desired target location from the user
            
            % Display Instructions to the User
            msgHandle = msgbox('Please select the target location using the left mouse button');
            uiwait(msgHandle,5); % wait for user to click 'OK' or wait 5 seconds
            if (ishandle(msgHandle) == 1) % if the msgbox still exists, close it
                delete(msgHandle);
            end
            
            % Get the Desired Target Location on the Map
            title('Please select the target location using the left mouse button');
            buttonPressed = 0;
            while (buttonPressed ~= map.window.leftMouseButton) % while the left mouse button hasn't been pressed
                [xLocation,yLocation,buttonPressed] = ginput(1); % get user click position
            end
            
            map.inputs.targetLocation = floor([xLocation,yLocation]); % store the location of the target
            
            % Display Target Location on the Map
            map.data(map.inputs.targetLocation) = map.legend.target;
            plot(map.inputs.targetLocation(1) + 0.5, map.inputs.targetLocation(2) + 0.5,'gd');
            text(map.inputs.targetLocation(1) + 1,map.inputs.targetLocation(2) + 0.5,'Target');
            
        end
        
        function selectObstacles(map)
        % EXAMPLE FUNCTION CALL: selectObstacles(map);
        % PROGRAMMER: Frederick Wachter
        % DATE CREATED: 2016-06-01
        % PURPOSE: Get desired obstacle locations from the user
            
            % Display Instructions to the User
            msgHandle = msgbox('Please select obstacle locations using the left mouse button, to select the last obstacle use the right mouse button');
            uiwait(msgHandle,5); % wait for user to click 'OK' or wait 5 seconds
            if (ishandle(msgHandle) == 1) % if the msgbox still exists, close it
                delete(msgHandle);
            end
            
            % Get the Desired Obstacle Locations from the Map
            title('Please select obstacle locations using the left mouse button, to select the last obstacle use the right mouse button','Color','blue');
            buttonPressed = 1;
            while (buttonPressed == map.window.leftMouseButton) % while the left button is being pressed per mouse click
                [xLocation,yLocation,buttonPressed] = ginput(1); % get user click position
                map.data(floor(xLocation),floor(yLocation)) = map.legend.obstacle; % store the obstacle location into the map
                plot(floor(xLocation) + 0.5,floor(yLocation) + 0.5,'ro'); % plot the location on the map
                map.inputs.obstacles = map.inputs.obstacles + 1; % increment the amount of obstacles
            end
            
        end
        
        function selectStart(map)
        % EXAMPLE FUNCTION CALL: selectStart(map);
        % PROGRAMMER: Frederick Wachter
        % DATE CREATED: 2016-06-01
        % PURPOSE: Get desired start location from the user
            
            % Display Instructions to the User
            msgHandle = msgbox('Please select the start location using the left mouse button');
            uiwait(msgHandle,5); % wait for user to click 'OK' or wait 5 seconds
            if (ishandle(msgHandle) == 1) % if the msgbox still exists, close it
                delete(msgHandle);
            end
            
            % Get the Desired Start Location on the Map
            title('Please select the start location using the left mouse button');
            buttonPressed = 0;
            while (buttonPressed ~= map.window.leftMouseButton) % while the left mouse button hasn't been pressed
                [xLocation,yLocation,buttonPressed] = ginput(1); % get user click position
            end
            
            map.inputs.startLocation = floor([xLocation,yLocation]); % store the location of the start
            
            % Display Start Location on the Map
            map.data(map.inputs.targetLocation) = map.legend.start;
            plot(map.inputs.startLocation(1) + 0.5,map.inputs.startLocation(2) + 0.5,'bo');
            text(map.inputs.startLocation(1) + 1,map.inputs.startLocation(2) + 0.5,'Start');
            
        end
        
        function exportData(map)
        % EXAMPLE FUNCTION CALL: exportData(map);
        % PROGRAMMER: Frederick Wachter
        % DATE CREATED: 2016-06-01
        % PURPOSE: Export map data
            
            export.map.data = map.data;
            export.map.legend = map.legend;
            
            [fileName,pathName] = uiputfile('map.mat','Save Map');
            save([pathName,'/',fileName],'-struct','export','map');
            
        end
        
    end


end


