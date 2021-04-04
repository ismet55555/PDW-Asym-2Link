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
% This model simulates a passive dynamic walker model (PDW), walking on
% level or angled ground. The walker can walk on any angle slope.
%
% Please see below to overview which options and paramters are avilable and
% can be changed



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
log.show_time     = true;
log.show_ms       = true;
log.default_level = 4; 


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%  Simulation Parameters  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
p.sim.dt			= 0.001;			% Simulation time step (s)
p.sim.total_strides = 15;				% Total Steps Taken
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
p.walker.right.ms1R = 0.050000;  % Lower right shan                k 


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
p.walker.right.rRb = -0.00;
p.walker.right.dR  = 0.00;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  Joint Properties   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% NOTE: This section currently does not work and is under construction
%       The walker does not currently respond to changes in thre paramters
%       In this section
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
% Walker masses
num_of_divs = 5;
VAR_mh   = linspace( 1.00, 3.00, num_of_divs)

VAR_mt1L = linspace( 0.75, 1.25, num_of_divs)
VAR_ms1L = linspace( 0.01, 0.50, num_of_divs)

VAR_mt1R = linspace( 0.75, 1.25, num_of_divs)
VAR_ms1R = linspace( 0.01, 0.50, num_of_divs)


% Walker distances
num_of_divs = 3;
perc_down = 0.85;
perc_up = 1.15;
VAR_a1L = linspace( 0.333 * perc_down, 0.333 * perc_up, num_of_divs)
VAR_b1L = linspace( 0.333 * perc_down, 0.333 * perc_up, num_of_divs)
VAR_c1L = linspace( 0.333 * perc_down, 0.333 * perc_up, num_of_divs)

VAR_a1R = linspace( 0.333 * perc_down, 0.333 * perc_up, num_of_divs)
VAR_b1R = linspace( 0.333 * perc_down, 0.333 * perc_up, num_of_divs)
VAR_c1R = linspace( 0.333 * perc_down, 0.333 * perc_up, num_of_divs)


% Walker foot
num_of_divs = 3;

VAR_rLa = linspace(-0.25, 0.25, num_of_divs)
VAR_rLb = linspace(-0.25, 0.25, num_of_divs)
VAR_dL  = linspace( 0.00, 0.05, num_of_divs)

VAR_rRa = linspace(-0.20, 0.20, num_of_divs)
VAR_rRb = linspace(-0.20, 0.20, num_of_divs)
VAR_dR  = linspace( 0.00, 0.05, num_of_divs)




% Calculating total iterations
total_iterations = length(VAR_mh) ...
                * length(VAR_mt1L) ...
                * length(VAR_ms1L) ...
                * length(VAR_mt1R) ...
                * length(VAR_ms1R) ...
                * length(VAR_a1L) ...
                * length(VAR_b1L) ...
                * length(VAR_c1L) ...
                * length(VAR_a1R) ...
                * length(VAR_b1R) ...
                * length(VAR_c1R) ...
                * length(VAR_rLa) ...
                * length(VAR_rLb) ...
                * length(VAR_dL) ...
                * length(VAR_rRa) ...
                * length(VAR_rRb) ...
                * length(VAR_dR);
log.fatal(sprintf('Total iterations: %i', total_iterations))
fprintf('\n\n\n')
iteration_number = 0;

% Starting iteration timer
iteration_timer = tic;

for index_1 = 1 : length(VAR_mh)
for index_2 = 1 : length(VAR_mt1L)
for index_3 = 1 : length(VAR_ms1L)
for index_4 = 1 : length(VAR_mt1R)
for index_5 = 1 : length(VAR_ms1R)
    
for index_6 = 1 : length(VAR_a1L)
for index_7 = 1 : length(VAR_b1L)
for index_8 = 1 : length(VAR_c1L)
for index_9 = 1 : length(VAR_a1R)
for index_10 = 1 : length(VAR_b1R)
for index_11 = 1 : length(VAR_c1R)

for index_12 = 1 : length(VAR_rLa)
for index_13 = 1 : length(VAR_rLb)
for index_14 = 1 : length(VAR_dL)
for index_15 = 1 : length(VAR_rRa)
for index_16 = 1 : length(VAR_rRb)
for index_17 = 1 : length(VAR_dR)
            iteration_number = iteration_number + 1;
            % Assigning variables
            
            % Walker masses
            p.walker.hip.mh = VAR_mh(index_1);
            p.walker.left.mt1L = VAR_mt1L(index_2);
            p.walker.left.ms1L = VAR_ms1L(index_3);
            p.walker.right.mt1R = VAR_mt1R(index_4);
            p.walker.right.ms1R = VAR_ms1R(index_5);

            % Walker distances
            p.walker.left.a1L = VAR_a1L(index_6);
            p.walker.left.b1L = VAR_b1L(index_7);
            p.walker.left.c1L = VAR_c1L(index_8);
            p.walker.right.a1R = VAR_a1R(index_9);
            p.walker.right.b1R = VAR_b1R(index_10);
            p.walker.right.c1R = VAR_c1R(index_11);

            % Walker foot
            p.walker.left.rLa = VAR_rLa(index_12);
            p.walker.left.rLb = VAR_rLb(index_13);
            p.walker.left.dL = VAR_dL(index_14);
            p.walker.right.rRa = VAR_rRa(index_15);
            p.walker.right.rRb = VAR_rRb(index_16);
            p.walker.right.dR = VAR_dR(index_17);



            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %Start Simulation Timer
            tic
            log.info('Simulation started')
            log.info(sprintf('Number of strides: %i', p.sim.total_strides))

            %Calling Walker_Control function to start simulation
            [p, results] = Walker_Control(p);

            % Log how long it took to run the single simulation run
            results.sim.run_time = toc;
            log.info('Simulation complete')
            log.info(sprintf('Simulation duration: %4.2fs', results.sim.run_time))

            % Plot specified result plots
            log.info('Plotting any specified plots ...');
            plot_results(p, results)

            %Saving Simulation Data
            log.info('Saving simulation data to file ...');
            save_parameters(p, results)

            log.info('Simulation run complete')
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            
            log.fatal(sprintf('[ %i of %i (%4.2f) ] - Duration: %4.2fs - Step: %i - Success: %i', ...
                iteration_number, ...
                total_iterations, ...
                (iteration_number / total_iterations) * 100, ...
                results.sim.run_time, ...
                results.sim.step, ...
                results.sim.success))
end
end
end
end
end
end
end
end
end
end
end
end
end
end
end
end
end

fprintf('\n\n\n')
fprintf('=========================================================')
fprintf('\n\n\n')
log.fatal(sprintf('Total duration: %4.2fs', toc(iteration_timer)))
log.fatal(sprintf('Mean duration per iteration: %4.3fs', toc(iteration_timer) / iteration_number))
fprintf('\n\n')
fprintf('=========================================================')
fprintf('\n\nDONE\n\n')











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
