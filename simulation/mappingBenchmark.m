%% ---- PROGRAM INFORMAITON ----
% PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
% DATE CREATED: 2016-12-17
% PURPOSE: Benchmark script for mapping algoirthms

% Please refer to the Wiki for instructions on how to use this class
% GITHUB WIKI: https://github.com/FWachter/Micromouse/wiki/MATLAB

% NOTE: The change directory portion of this script was meant for MATLAB on
% a Mac OS not Windows

%% Initialize Variables
tests = 0;
fileName = 'Test1_nodeMapping_161219';

%% Run Tests
totalMovements = zeros(tests, 1);
for i = 1:tests
    clear sim nodes robot;
    node_Mapping;
    totalMovements(i) = sim.robot.movements;
end

%% Change Directory to Log File Directory
filePath = mfilename('fullpath');
removalIndex = find(filePath == '/', 2, 'last');
cd(filePath(1:removalIndex-1));
cd logFiles;

%% Write to File
csvwrite(fileName, totalMovements);