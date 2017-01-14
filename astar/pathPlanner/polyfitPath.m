%% ---- PROGRAM INFORMAITON ----
% PROGRAMMER: Frederick Wachter
% DATE CREATED: 2016-06-01
% PURPOSE: Show polynomial path fit to solution path with different turn agression
% CONTACT INFO: wachterfreddy@gmail.com

% Please refer to the Wiki for instructions on how to use this script
% GITHUB WIKI: https://github.com/FWachter/Micromouse/wiki/MATLABs

cutCorners;
x = newPath(:,1);
y = newPath(:,2);

tic
interpolants = 3;
xx = linspace(x(end),x(end-1),interpolants);
yy = linspace(y(end),y(end-1),interpolants);
for index = (length(x)-2):-1:1
    xx = [xx(1:end-1),linspace(x(index+1),x(index),interpolants)];
    yy = [yy(1:end-1),linspace(y(index+1),y(index),interpolants)];
end
toc

windowWidth = 15;

astar.displayMap;
title('Fifth Order Polynomial Path Fit');
polynomialOrder = 5;
smoothX = sgolayfilt(xx, polynomialOrder, windowWidth);
smoothY = sgolayfilt(yy, polynomialOrder, windowWidth);
plot(smoothX+0.5,smoothY+0.5,'k');

astar.displayMap;
title('Fourth Order Polynomial Path Fit');
polynomialOrder = 4;
smoothX = sgolayfilt(xx, polynomialOrder, windowWidth);
smoothY = sgolayfilt(yy, polynomialOrder, windowWidth);
plot(smoothX+0.5,smoothY+0.5,'k');

astar.displayMap;
title('Third Order Polynomial Path Fit');
polynomialOrder = 3;
smoothX = sgolayfilt(xx, polynomialOrder, windowWidth);
smoothY = sgolayfilt(yy, polynomialOrder, windowWidth);
plot(smoothX+0.5,smoothY+0.5,'k');


