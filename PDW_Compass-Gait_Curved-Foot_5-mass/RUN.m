%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%				      Compass Gait PDW Model with Curved Feet
%						         Ismet Handzic
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%  Simulation Description  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% TODO ...
%
% Go back and do left , right, per step energy
% Decide on saved parameters
% Finish per step plots
% Per step 
%   - overlap plot
%   - averages of interppolated?  dashed line...



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%  Setup Stripts and Utilities  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Mapping directories
if ~setup()
   return 
end

% Setting up message logger object
global log
log = logger();
log.show_time = true;
log.show_ms   = true;



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%  Simulation Parameters  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
p.sim.dt			= 0.001;			% Simulation time step (s)
p.sim.total_strides = 10;				% Total Steps Taken
p.sim.g             = 9.80665;			% Gravity (m/s^2)
p.sim.theta         = asin(8.55/144);	% Walking ramp Angle (rad)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  Masses  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Hip (kg)
p.walker.hip.mh = 2.000000;

% Left Leg (kg)
p.walker.left.mt1L = 1.000000;	% Upper left thigh
p.walker.left.ms1L = 0.050000;	% Lower left shank 

% Right Leg (kg)
p.walker.right.mt1R = 1.000000;  % Upper right thigh 
p.walker.right.ms1R = 0.050000;  % Lower right shank 


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  Mass Position  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Left Leg (m)
p.walker.left.a1L = 1/3;
p.walker.left.b1L = 1/3;
p.walker.left.c1L = 1/3;

% Right Leg (m)
p.walker.right.a1R = 1/3;
p.walker.right.b1R = 1/3;
p.walker.right.c1R = 1/3;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  Foot Shape  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Foot Definition Equation:  footRadius = rLa + rLb*angle
% Left Foot (m)
p.walker.left.rLa = 1/3;     
p.walker.left.rLb = -0.00;
p.walker.left.dL  = 0.00;

% Right Foot (m)
p.walker.right.rRa = 1/3;
p.walker.right.rRb = -0.05;
p.walker.right.dR  = 0.00;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  Joint Properties   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Joint Damping (N*m*s/rad)
p.walker.joints.damping_constant.left  = 0;
p.walker.joints.damping_constant.right = 0;

% Joint Stiffness (N*m/rad)
p.walker.joints.stiffness_constant.left  = 0;
p.walker.joints.stiffness_constant.right = 0;

% Torque Input (N*m)
p.walker.joints.torque_input.left  = 0;
p.walker.joints.torque_input.right = 0;

% Joint Angular Position Limits (rad)
p.walker.joints.limits.ang_pos.left  = [-pi/2, pi/2];
p.walker.joints.limits.ang_pos.right = [-pi/2, pi/2];

% Joint ANgular Velocity Limits (rad/s)
p.walker.joints.limits.ang_vel.left  = [-999, 999];
p.walker.joints.limits.ang_vel.right = [-999, 999];


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  Initial Conditions  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% NOTE: Left leg is the first stance leg.
%       So to start off, q1, qd1 are for the left stance leg 
%       and q2 and qd2 are for the swinging right leg.

% Angular Positions for both legs (rad)
p.walker.init.q1 =  0.057700106303669;
p.walker.init.q2 = -0.418437140111164;

% Angular Velocities for both legs (rad/s)
p.walker.init.qd1 = -0.901439041094875;
p.walker.init.qd2 = -0.039885255927632;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%  Simulation Display and Output  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
p.sim.output.animation		= false;  % Simulation Animation
p.sim.output.energy         = false;  % Energy Plot
p.sim.output.force			= false;  % Kinetic/Forces Plot
p.sim.output.step_length	= false;  % Step Length Plot
p.sim.output.limit_cycle	= false;  % Limit Cycle Plot
p.sim.output.angle			= false;  % Leg and Hip Angle Plot
p.sim.output.angular_vel	= false;  % Legs Angular Velocity Plot
p.sim.output.angular_accel	= false;  % Legs Angular Acceleration Plot


% Options to save each animation frame to file
% NOTE: These options are only relevant if "p.sim.output.figures.animation" 
%       is set to "true"
p.sim.output.save_walking_frames        = false;
p.sim.output.save_walking_frames_dir    = "walking_frames";
p.sim.output.print_frame_index          = 0;
p.sim.output.frames_to_video            = true;  % TODO: Compile into a video at the end


% Data Collection 
p.sim.output.result_output_filename	= 'DATA.txt'; % String to use as .txt file name
p.sim.output.save_plots             = true;        % Option to save output plots
p.sim.output.result_dir             = "Output/data/";  % Directory where data will be stored


% Walker Animation Display Mass Multipliers
p.walker.animation.mh_mul = 50;
p.walker.animation.mt_mul = 50;
p.walker.animation.ms_mul = 50;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  Simulation   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Start Simulation Timer
tic
log.info('Simulation started')
log.info(sprintf('Number of strides: %i', p.sim.total_strides))
log.info(sprintf('Estimated run duration: %.2fs', (0.2452*p.sim.total_strides + 4.6282)*1.05))

%Calling Walker_Control function to start simulation
[p, results] = Walker_Control(p);

% Log how long it took to run the single simulation run
results.sim.run_time = toc;
log.info('Simulation complete')
log.info(sprintf('Run duration: %4.2fs', results.sim.run_time))

% Generating a report
% TODO: Option to save formatted report to text file
% generate_report(p, results)

% Plot specified result plots
log.info('Plotting any specified plots ...');
plot_results(p, results)

%Saving Simulation Data
log.info('Saving simulation data to file ...');
save_parameters(p, results)

log.info('Simulation run complete')







%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [success] = setup()
    % Funtion description TODO
    try
        clc			% Clear the command window
        close all	% Close all open figures and plots
        clear all   % Clear all variables in the workspace

        % Reset the path definitions
        path(pathdef); 
        
        % Adding all needed paths
        addpath(genpath(mfilename));
        paths = ["Simulation", "Dependents", "Utility"];
        for p = 1 : length(paths)
             % Check if referenced directories exist
            if ~exist(paths(p), 'dir')
                fprintf(2, "Failed to map the following directory path: '%s'\n\n", paths(p))
                success = false;
                return
            end
            
            % Add directory to path reference
            addpath(genpath(char(paths(p))));
        end
        
        success = true;
    catch e
        fprintf(2, "\n\nFailed to set up simulation. Exception: %s\n\n", e.message)
        success = false;
    end
end
