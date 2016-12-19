%% ---- PROGRAM INFORMAITON ----
% PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
% DATE CREATED: 2016-12-14
% PURPOSE: Node structure class

% Please refer to the Wiki for instructions on how to use this class
% GITHUB WIKI: https://github.com/FWachter/Micromouse/wiki/MATLAB


classdef node < handle
    
    properties
        goal
        commands
    end
    
    properties(SetAccess = private, GetAccess = private)
        stack
        tracker
    end

%% CONSTRUCTOR METHOD
    methods
        
        function nodes = node(varargin)
        % EXAMPLE FUNCTION CALL: nodes = node(varargin)
        % PROGRAMMER: Frederick Wachter
        % DATE CREATED: 2016-12-14
        % PURPOSE: Construct the structure
            
            % Initialize Goal Properties
            nodes.goal.location = [];
            
            % Initialize Command Properties
            nodes.commands = {};
            
            % Initialize Stack Properties
            nodes.stack.size = 0;
            nodes.stack.data = [];
            nodes.stack.map  = [];
            
            % Initialize Tracker Properties
            nodes.tracker.size       = 0;
            nodes.tracker.data       = [];
            nodes.tracker.directions = [];
            nodes.tracker.stackIndex = [];
            
        end
        
    end
    
%% PUBLIC METHODS

    methods
        
        function addNode(nodes, location, openDirections)
        % EXAMPLE FUNCTION CALL: nodes.addNode(location, openDirections)
        % PROGRAMMER: Frederick Wachter
        % DATE CREATED: 2016-12-14
        % PURPOSE: Adds node to node structure
            
            if ~(nodes.checkNodeExists(location))
                nodes.addToStack(location);
                nodes.addToTracker(location, openDirections);
            else
                % remove open direction
                warning('Node already exists');
            end
            
            command = strcat('[AN] [', num2str(location), '] [', num2str(openDirections) ,']\n');
            nodes.addCommand(command);
            
        end
        
        function addStack(nodes, location, mapIndex)
        % EXAMPLE FUNCTION CALL: nodes.popStack(nodes)
        % PROGRAMMER: Frederick Wachter
        % DATE CREATED: 2016-12-14
        % PURPOSE: Adds location to stack
            
            nodes.addToStack(location, mapIndex);
            
            command = strcat('[AS] [', num2str(location), ']\n');
            nodes.addCommand(command);
            
        end

        function [location, mapIndex, stackIndex] = popStack(nodes)
        % EXAMPLE FUNCTION CALL: nodes.popStack(nodes)
        % PROGRAMMER: Frederick Wachter
        % DATE CREATED: 2016-12-14
        % PURPOSE: Remove last node in node stack
            
            if (nodes.stack.size == 0)
                error('Cannot pop stack when the stack is empty\n');
            end
            
            location = nodes.stack.data(end, :);
            mapIndex = nodes.stack.map(end);
            stackIndex = nodes.tracker.stackIndex(nodes.stack.map(end));
            
            nodes.removeLastStackElement();
            
            command = strcat('[RS]\n');
            nodes.addCommand(command);
            
        end
        
        function removePreviousNode(nodes)
        % EXAMPLE FUNCTION CALL: nodes.removePreviousNode()
        % PROGRAMMER: Frederick Wachter
        % DATE CREATED: 2016-12-14
        % PURPOSE: Removes node at robots current location from node structure
            
            location = nodes.stack.data(end, :);
        
            nodes.removeLastStackElement();
            nodes.removeLastTrackerElement();
            
            command = strcat('[RN] [', num2str(location), ']\n');
            nodes.addCommand(command);
            
        end
        
        function removeNodeDirection(nodes, direction)
        % EXAMPLE FUNCTION CALL: nodes.removeNodeDirection(location, direction)
        % PROGRAMMER: Frederick Wachter
        % DATE CREATED: 2016-12-14
        % PURPOSE: Remove movement direction from specified node
        
            location = nodes.stack.data(end, :);
            mapIndex = nodes.stack.map(end);
            nodes.removeDirection(mapIndex, direction);
            
            command = strcat('[RD] [', num2str(direction), '] [', num2str(location), ']\n');
            nodes.addCommand(command);
            
        end
        
        function location = getPreviousNodeLocation(nodes)
        % EXAMPLE FUNCTION CALL: nodes.getPreviousNodeLocation()
        % PROGRAMMER: Frederick Wachter
        % DATE CREATED: 2016-12-14
        % PURPOSE: Get the last element from node stack
            
            location = nodes.stack.data(end, :);
            
        end
        
        function openDirections = getPreviousNodeOpenDirections(nodes)
        % EXAMPLE FUNCTION CALL: nodes = getPreviousNodeOpenDirections()
        % PROGRAMMER: Frederick Wachter
        % DATE CREATED: 2016-12-14
        % PURPOSE: Get the last element open directions from node stack
            
            mapIndex = nodes.stack.map(end);
            openDirections = nodes.tracker.directions(mapIndex, :);
            
        end
        
        function openDirections = getNodeOpenDirections(nodes, index)
        % EXAMPLE FUNCTION CALL: nodes = getNodeOpenDirections(index)
        % PROGRAMMER: Frederick Wachter
        % DATE CREATED: 2016-12-14
        % PURPOSE: Get open directions of a node at specified index
            
            openDirections = nodes.tracker.directions(index, :);
            
        end
        
        function [nodeExists, index, stackIndex] = checkNodeExists(nodes, location)
        % EXAMPLE FUNCTION CALL: nodes.checkNodeExists(location)
        % PROGRAMMER: Frederick Wachter
        % DATE CREATED: 2016-12-14
        % PURPOSE: Check if a specified node already exists
            
            nodeExists = 0;
            for index = 1:nodes.tracker.size
                if (sum(location == nodes.tracker.data(index, :)) == 2)
                    nodeExists = 1;
                    break;
                end
            end
            stackIndex = nodes.tracker.stackIndex(index);
            
        end
        
        function exportCommands(nodes, fileName)
        % EXAMPLE FUNCTION CALL: nodes.exportCommands(fileName)
        % PROGRAMMER: Frederick Wachter
        % DATE CREATED: 2016-12-19
        % PURPOSE: Exports command executed in node class
            
            fileID = fopen(fileName, 'w');
            fprintf(fileID, 'Exported commands from node class\n\n');
            fprintf(fileID, '_____ Legend _____\n');
            fprintf(fileID, '[AN] [location] [open directions] - add node\n');
            fprintf(fileID, '[AS] [location] - add to stack\n');
            fprintf(fileID, '[RS] - remove from stack\n');
            fprintf(fileID, '[RN] [location] - remove node\n');
            fprintf(fileID, '[RD] [direction] [location] - remove direction from node\n\n');
            fprintf(fileID, 'Note: RN and RD are only called when on the node, not randomly\n\n');
            for line = 1:length(nodes.commands)
                fprintf(fileID, nodes.commands{line});
            end
            
            fprintf('Export successful');
            
        end
        
    end 

%% PRIVATE METHODS

    methods (Access = private)
        
        % Node Addition/Removal Functions
        
        function addToStack(nodes, location, mapIndex)
        % EXAMPLE FUNCTION CALL: nodes.addToStack(location)
        % PROGRAMMER: Frederick Wachter
        % DATE CREATED: 2016-12-14
        % PURPOSE: Add specified node to node stack
            
            nodes.stack.size = nodes.stack.size + 1;
            nodes.stack.data = [nodes.stack.data; location];
            if (exist('mapIndex', 'var'))
                nodes.stack.map(nodes.stack.size) = mapIndex;
            else
                nodes.stack.map(nodes.stack.size)  = nodes.tracker.size+1;
            end
            
        end
        
        function addToTracker(nodes, location, openDirections)
        % EXAMPLE FUNCTION CALL: nodes.addToTracker(location, openDirections)
        % PROGRAMMER: Frederick Wachter
        % DATE CREATED: 2016-12-14
        % PURPOSE: Add specified node to node tracker
            
            nodes.tracker.size = nodes.tracker.size + 1;
            nodes.tracker.data = [nodes.tracker.data; location];
            nodes.tracker.directions = [nodes.tracker.directions; openDirections];
            nodes.tracker.stackIndex = [nodes.tracker.stackIndex; nodes.stack.size];
            
        end
        
        function removeLastStackElement(nodes)
        % EXAMPLE FUNCTION CALL: nodes.removeLastStackElement()
        % PROGRAMMER: Frederick Wachter
        % DATE CREATED: 2016-12-14
        % PURPOSE: Remove last element from node stack
            
            nodes.stack.size = nodes.stack.size - 1;
            nodes.stack.data = nodes.stack.data(1:(end-1), :);
            nodes.stack.map  = nodes.stack.map(1:(end-1));
            
        end
        
        function removeLastTrackerElement(nodes)
        % EXAMPLE FUNCTION CALL: nodes .removeLastTrackerElement()
        % PROGRAMMER: Frederick Wachter
        % DATE CREATED: 2016-12-14
        % PURPOSE: Remove last element from node tracker
            
            nodes.tracker.size = nodes.tracker.size - 1;
            nodes.tracker.data = nodes.tracker.data(1:(end-1), :);
            nodes.tracker.directions = nodes.tracker.directions(1:(end-1), :);
            nodes.tracker.stackIndex = nodes.tracker.stackIndex(1:(end-1), :);
            
        end
        
        function removeDirection(nodes, index, direction)
        % EXAMPLE FUNCTION CALL: nodes.removeDirection(index, direction)
        % PROGRAMMER: Frederick Wachter
        % DATE CREATED: 2016-12-14
        % PURPOSE: Remove movement direction from specified node
            
            nodes.tracker.directions(index, direction) = 0;
            
        end
        
        function addCommand(nodes, command)
        % EXAMPLE FUNCTION CALL: nodes.addCommand(command)
        % PROGRAMMER: Frederick Wachter
        % DATE CREATED: 2016-12-19
        % PURPOSE: Add command to list of performed commands
            
            nodes.commands = [nodes.commands, command];
            
        end
        
    end
    
end


