close all
clc
clear all

%===========================================
%DEFINES DATA AND STARTS SIMULATION PROGRAM
%============================================

% Design Parameters - On diagram

% TODO: CONVERT TO STRUCTS!!!


%   Masses
mh = 4.0000001;

ms1L = 0.0000000001;   %low shank left
ms2L = 0.0000000001;   %up shank left

mt1L = 1.00000001;   %up thigh left
mt2L = 0.100000001;  %low thigh left

ms1R = 0.0000000001;  %low shank right 
ms2R = 0.0000000001;  %up shank right

mt1R = 1.00000001;   %up thigh right
mt2R = 0.10000001;   %low thigh right

%   Left Leg
a1L = 0.0;
b1L = 0.0;
c1L = 0.0;

a2L = 0.2;
b2L = 0.2;
c2L = 0.2;

%   Right Leg
a1R = 0.0;
b1R = 0.0;
c1R = 0.0;

a2R = 0.2;
b2R = 0.2;
c2R = 0.2;

%Derived
%Left leg
lsL = a1L + b1L + c1L;   %%%ANOTATE BY FIGURE
ltL = a2L + b2L + c2L;
LL  = lsL + ltL;

lsR = a1R + b1R + c1R;
ltR = a2R + b2R + c2R;
LR  = lsR + ltR;

% Gravity
g = 9.81;

% Time Step
dt = 0.0001;

% Ramp Angle
theta = asin(8.5/144);    

% Total Steps Taken
total_steps = 15;

% Intial Angular Position for legs
q = [0.057700106303669; -0.418437140111164; 0];

% Intial Angular Velocities for legs
qd = [-0.901439041094875; -0.039885255927632; 0];


% Display Conditions
display = 1;  %if you want walker display, actual walking figure
time_plot = 1;
energy = 0;  %energy plot
step_length = 0;
cycle = false;  %limit cycle plot
save_walking_figure = 0;
velocity_disp = 0;   %display velocities in command window
print_count = 0;

% Walker Display Mass Multipiers
mhmul = 20;
mtmul = 20;
msmul = 20; 

% Data Collection
changed_variable = ms1L;
params.data_name = 'eight_coarse';  %string to use to text file (make sure to create folder with identical name in the same directory)
params.other     = 'other';         %incase you want to make another string for something (unused)


tic 
    %--------------------------------------------------- 
    %loop this part if you want to make a parameter a variable

	
    %prep to send into other functions (m-files)
    parameters = [mh, mt1R, mt2R, ms1R, ms2R, a1R, b1R, c1R, a2R, b2R, c2R,lsR, ltR, LR,...
                  mt1L, mt2L, ms1L, ms2L, a1L, b1L, c1L, a2L, b2L, c2L, lsL, ltL, LL, ...
				  g, dt, theta, ...
                  time_plot, total_steps, display, energy, step_length,cycle,...
                  save_walking_figure, print_count, mhmul,mtmul, msmul, changed_variable, velocity_disp];
    
    %call run_walker function and return values
    [stop_condition, steps, successful, right_step_length, left_step_length] = run_walker(q, qd, parameters, params, total_steps);
    
	
    % Open textfile for data
    fid = fopen(strcat(['test', '.txt']),'a');  
    
    % Write all parameter to text file to use later for analysis in MATLAB with dlmread function
    all_params = [mh ms1L ms2L mt1L mt2L a1L b1L c1L a2L b2L c2L LL lsL ltL ...
		          ms1R ms2R mt1R mt1R a1R b1R c1R a2R b2R c2R LR lsR ltR ...
		          steps mean(left_step_length) mean(right_step_length) stop_condition successful];
    fprintf(fid, '%6.4f\t%6.4f\t%6.4f\t%6.4f\t%6.4f\t%6.4f\t%6.4f\t%6.4f\t%6.4f\t%6.4f\t%6.4f\t%6.4f\t%6.4f\t%6.4f\t%6.4f\t%6.4f\t%6.4f\t%6.4f\t%6.4f\t%6.4f\t%6.4f\t%6.4f\t%6.4f\t%6.4f\t%6.4f\t%6.4f\t%6.4f\t%6.4f\t%6.4f\t%6.4f\t%6.4f\t%6.4f\t\r',all_params);
    
    % close text file
    fclose(fid);
    
    %-----------------------------------------------------

toc
