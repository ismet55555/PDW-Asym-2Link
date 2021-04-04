function save_parameters(p, results)
% SAVE_PARAMETERS: This script specifies, organizes, and saves simulation
%                  parameters to file

global log

% Define the full path to the data file
full_path = strcat(p.sim.output.result_dir, p.sim.output.result_output_filename);
log.debug(sprintf('Data output file path: %s', full_path))

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%  Define Saved Parameters  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Selected individual parameters to be saved
% Simulation
saved_parameters.theta          = p.sim.theta;
saved_parameters.g              = p.sim.g;
saved_parameters.dt             = p.sim.dt;
saved_parameters.total_strides  = p.sim.total_strides;


% Initial Conditions
saved_parameters.q1             = p.walker.init.q1;
saved_parameters.q2             = p.walker.init.q2;
saved_parameters.qd1            = p.walker.init.qd1;
saved_parameters.qd2            = p.walker.init.qd2;


% Walker 
saved_parameters.mh             = p.walker.hip.mh;

saved_parameters.mt1L           = p.walker.left.mt1L;
saved_parameters.ms1L           = p.walker.left.ms1L;
saved_parameters.a1L            = p.walker.left.a1L;
saved_parameters.b1L            = p.walker.left.b1L;
saved_parameters.c1L            = p.walker.left.c1L;
saved_parameters.rLa            = p.walker.left.rLa;
saved_parameters.rLb            = p.walker.left.rLb;
saved_parameters.dL             = p.walker.left.dL;
saved_parameters.LL             = p.walker.left.LL;

saved_parameters.mt1R           = p.walker.right.mt1R;
saved_parameters.ms1R           = p.walker.right.ms1R;
saved_parameters.a1R            = p.walker.right.a1R;
saved_parameters.b1R            = p.walker.right.b1R;
saved_parameters.c1R            = p.walker.right.c1R;
saved_parameters.rRa            = p.walker.right.rRa;
saved_parameters.rRb            = p.walker.right.rRb;
saved_parameters.dR             = p.walker.right.dR;
saved_parameters.LR             = p.walker.right.LR;
		

% Results - General
saved_parameters.run_time       = results.sim.run_time; 
saved_parameters.duration       = results.sim.duration; 
saved_parameters.stopped        = results.sim.stopped;
saved_parameters.success        = results.sim.success; 
saved_parameters.stride         = results.sim.stride; 
saved_parameters.step           = results.sim.step;
saved_parameters.duration       = results.sim.duration; 
saved_parameters.fail           = results.fail.fail;
saved_parameters.phase          = results.fail.phase;

% If there first step did not even produce dynamics
if not(isfield(results.energy.left_right,'total'))
    % Return without saving the results
    return
end

% Energy
saved_parameters.energy_all_total_mean = mean(results.energy.left_right.total.mean(1:end));
saved_parameters.energy_all_KE_mean    = mean(results.energy.left_right.KE.mean(1:end));
saved_parameters.energy_all_PE_mean    = mean(results.energy.left_right.PE.mean(1:end));

saved_parameters.energy_left_total_mean = mean(results.energy.left_right.total.mean(1:2:end));
saved_parameters.energy_left_KE_mean    = mean(results.energy.left_right.KE.mean(1:2:end));
saved_parameters.energy_left_PE_mean    = mean(results.energy.left_right.PE.mean(1:2:end));

saved_parameters.energy_right_total_mean = mean(results.energy.left_right.total.mean(1 + 1:2:end));
saved_parameters.energy_right_KE_mean    = mean(results.energy.left_right.KE.mean(1 + 1:2:end));
saved_parameters.energy_right_PE_mean    = mean(results.energy.left_right.PE.mean(1 + 1:2:end));

% Forces
saved_parameters.force_left_Rx_mean = mean(results.force.left.Rx.mean(1:2:end));
saved_parameters.force_left_Ry_mean = mean(results.force.left.Ry.mean(1:2:end));
saved_parameters.force_left_Rx_max  = mean(results.force.left.Rx.max(1:2:end));
saved_parameters.force_left_Ry_max  = mean(results.force.left.Ry.max(1:2:end));
saved_parameters.force_left_Rx_min  = mean(results.force.left.Rx.min(1:2:end));
saved_parameters.force_left_Ry_min  = mean(results.force.left.Ry.min(1:2:end));

saved_parameters.force_right_Rx_mean = mean(results.force.right.Rx.mean(1 + 1:2:end));
saved_parameters.force_right_Ry_mean = mean(results.force.right.Ry.mean(1 + 1:2:end));
saved_parameters.force_right_Rx_max  = mean(results.force.right.Rx.max(1 + 1:2:end));
saved_parameters.force_right_Ry_max  = mean(results.force.right.Ry.max(1 + 1:2:end));
saved_parameters.force_right_Rx_min  = mean(results.force.right.Rx.min(1 + 1:2:end));
saved_parameters.force_right_Ry_min  = mean(results.force.right.Ry.min(1 + 1:2:end));


% Other
saved_parameters.left_step_length_mean    = results.other.left.step_length.mean;
saved_parameters.left_step_length_median  = results.other.left.step_length.median;

saved_parameters.right_step_length_mean   = results.other.right.step_length.mean;
saved_parameters.right_step_length_median = results.other.right.step_length.median;


% TODO: Power


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  Data Formatting  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Creating a the formatting for file entry
field_names = fieldnames(saved_parameters);
parameters_format = "";
headers_format    = "";
for field = 1 : numel(field_names)
    headers_format    = strcat(headers_format,    "%s\t");
    parameters_format = strcat(parameters_format, "%6.5f\t");
end
headers_format    = strcat(headers_format,    "\r\n");
parameters_format = strcat(parameters_format, "\r\n");


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%  Create File with Headers  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Getting header names
headers = strings(size(field_names));
[headers{:}] = field_names{:};
headers = headers';

% Add the file with headers if it does not exist yet
% TODO: Create header if also file is empty
if ~isfile(full_path)
    log.debug(sprintf('Adding new data output file and adding column headers ...'))
    open_file = fopen(full_path, 'w');            % Create and open the new file
    fprintf(open_file, headers_format, headers);  % Add the headers
    fclose(open_file);                            % Close the file
end

% TODO: Add check to see if the length of the previously written parameters is the same
%       as the length of the current parameters saved 



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  Save the Data  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Create a vector containing all parameter values
saved_parameters_values = [];
for field = 1 : numel(field_names)
    parameter_value = saved_parameters.(field_names{field});
    saved_parameters_values = [saved_parameters_values, parameter_value];
end

% Write the data
log.debug(sprintf('Writing data to file ...'))
open_file = fopen(full_path, 'a');                               % Open text file to save model data
fprintf(open_file, parameters_format, saved_parameters_values);  % Write all parameter to text file
fclose(open_file);                                               % Close text file


end
