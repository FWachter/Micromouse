
% Michael Cunningham
% Simulation Driver
% 4/10/2017

clear; clc
close all;

dbstop if error
dbstop if warning

rng(1);

% State Vector
%   x       - "X" Position          [m]
%   y       - "Y" Position          [m]
%   theta   - Angle From X Axis     [rad]
%   vt      - Forward Velocity      [m/s]
%   wt      - Angular Velocity      [rad/s]

% Measurement Noise Vector
%   n1      - Measurement Noise STD, Right Distance [m]
%   n2      - Measurement Noise STD, Left Distance  [m]
%   n3      - Measurement Noise STD, Right Velocity [rad/s]
%   n4      - Measurement Noise STD, Left Velocity  [rad/s]

X0 = zeros(5,1);
N = [0.005; 0.005; 1; 1];

% Robot Constants
a = 10;
n = 0.05;
m = 0.03;
p = 0.02;
q = 0.03;
r = 0.01;
g = 1;
b = 10*a/g/r/20;
C = [a, b, r, g, m, n, p, q];

D = [0, 0, 0, 1
    0, 0, 0, 0.5
    0.90, 0.09, 0.09, 0.5
    0.99, 0.09, pi/2, 0.5
    0.99, 0.09, pi/2, 1];
pathconfig = [0; 0; 1; 0; 0];
pathtest = {@(x)x(1)>=0.75
    @(x)x(1)>=0.90
    @(x)x(2)>=0.09
    @(x)x(2)>=0.24
    @(x)x(2)>=0.99};

[X,Y,YD,Yconfig,T,U] = runSim(X0,N,C,D,pathconfig,pathtest);

PX = diag([(0.01)^2, (0.01)^2, (pi/18)^2, (1)^2, (1)^2]);
PV = diag([(0.1)^2, (0.1)^2]);
PN = diag([(0.005)^2, (0.005)^2, (2)^2, (2)^2]);

Xhat0 = X0+sqrt(diag(PX)).*randn(size(X0));

[Xhat,P,T] = runEstimator(Xhat0,PX,PV,PN,Y,YD,Yconfig,T(2:end),U,C);

figure; hold on; grid on;
plot(T,X(1,:)-Xhat(1,:),'r')
plot(T, 3*squeeze(sqrt(P(1,1,:))),'b')
plot(T,-3*squeeze(sqrt(P(1,1,:))),'b')
xlabel('Time [s]');
ylabel('X Error');
legend('error','3*sqrt(P)')

figure; hold on; grid on;
plot(T,X(2,:)-Xhat(2,:),'r')
plot(T, 3*squeeze(sqrt(P(2,2,:))),'b')
plot(T,-3*squeeze(sqrt(P(2,2,:))),'b')
xlabel('Time [s]');
ylabel('Y Error');
legend('error','3*sqrt(P)')

figure; hold on; grid on;
plot(T,X(3,:)-Xhat(3,:),'r');
plot(T, 3*squeeze(sqrt(P(3,3,:))),'b');
plot(T,-3*squeeze(sqrt(P(3,3,:))),'b');
xlabel('Time [s]');
ylabel('\theta Error');
legend('error','3*sqrt(P)');

figure; hold on; grid on;
plot(T,X(4,:)-Xhat(4,:),'r');
plot(T, 3*squeeze(sqrt(P(4,4,:))),'b');
plot(T,-3*squeeze(sqrt(P(4,4,:))),'b');
xlabel('Time [s]');
ylabel('V Error');
legend('error','3*sqrt(P)');

figure; hold on; grid on;
plot(T,X(5,:)-Xhat(5,:),'r');
plot(T, 3*squeeze(sqrt(P(5,5,:))),'b');
plot(T,-3*squeeze(sqrt(P(5,5,:))),'b');
xlabel('Time [s]');
ylabel('W Error');
legend('error','3*sqrt(P)');