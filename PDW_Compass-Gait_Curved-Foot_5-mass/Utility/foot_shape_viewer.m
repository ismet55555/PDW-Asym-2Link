%=========================================================================================
%
% Passive Dynamic Walker (PDW) Simulation Model
% Curved Foot Variation
%
%=========================================================================================
%
% Curved Foot Visualization Tool
%
%=========================================================================================
%
% This simple script aids to construct and visualize the walker's curved
% foot.
%
%=========================================================================================
% Author: Ismet Handzic
% Date  : 04/29/2020
%=========================================================================================

clc			% Clear the command window
close all	% Close all open figures and plots
clear all   % Clear all variables in the workspace


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%    Foot Shape Definition    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Foot definiton equation:   FootRadius = rA + rB*(Angle Range)
rA = 1/3;
rB = - 0.05;

% Define the leg to foot attachment offset
d = 0.0;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%    Deriving the Footshape    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Define the angle range of the foot shape definition
t_foot = linspace(-pi/2, pi/2, 100);

% Define the foot radius for each angle step
r_foot = zeros(1, length(t_foot));
for i = 1 : length(t_foot)
   r_foot(i) = rA + rB * (t_foot(i));
end

% Convert to Cartesian
[x_foot, y_foot] = pol2cart(t_foot - pi/2, r_foot);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%    Plotting the Footshape    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
foot_plot = figure;
hold on
% Foot Shape
plot(x_foot, y_foot, '-k', 'LineWidth', 2)
% Closing the foot shape
plot([-d, -d], [0, 1], '-k', 'LineWidth', 2)
% Leg Position
plot([x_foot(1), x_foot(end)], [y_foot(1), y_foot(end)], '-k', 'LineWidth', 2)
% Origin
plot(0, 0, '+b', 'MarkerSize', 30)

% Formatting Plot
set(foot_plot,'color', 'w');
grid on
box on
daspect([1, 1, 1])
title({"PDW Foot Shape Preview", sprintf("rA: %.2f - rB: %.2f - d: %.2f", rA, rB, d)})
xlabel('Distance (m)');
ylabel('Distance (m)')


