%% ---- PROGRAM INFORMAITON ----
% PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
% DATE CREATED: 2016-12-17
% PURPOSE: Benchmark script for mapping algoirthms

% Please refer to the Wiki for instructions on how to use this class
% GITHUB WIKI: https://github.com/FWachter/Micromouse/wiki/MATLAB

tests = 15;
totalMovements = zeros(tests, 1);

for i = 1:tests
    clear sim nodes robot;
    node_Mapping;
    totalMovements(i) = sim.robot.movements;
end

cd ~/Documents/Github/Micromouse/simulation/tests;
csvwrite('Test1_nodeMapping_161219', totalMovements);