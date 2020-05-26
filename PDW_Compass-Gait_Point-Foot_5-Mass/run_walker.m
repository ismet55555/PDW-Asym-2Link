%  Full Dynamics Model
%  This model describes a passive dynamic walker in terms of each individual legs
function [stop_condition, steps, successful, right_step_length, left_step_length] = ...
	run_walker(q, qd, parameters, params, total_steps)
% TODO: DESCRIBE FUNCTION ....


%Define all parameters by assigning them indecies of the incoming "parameters" 
LR = parameters(14);
LL = parameters(27);

total_steps			= parameters(32);
step_length			= parameters(35);
cycle				= parameters(36);
changed_variable	= parameters(42);

%Strings
data_name = params.data_name;

% Store the folder name and directory
data_path = strcat([data_name '/']);

% True if walker completes all total steps
successful = false;  

%Once it becomes 1 code will stop (opposite of successful variable)
stop_condition = 0; 

%creating blank matrix for step lengths
left_step_length  = [];
right_step_length = [];

% Loop steps
for steps = 1 : total_steps
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    Left Stance    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Calls dynamics of two link left 
	% (LEFT LEG IS ON THE GROUND, MECHANISM IS IN TWO LINK STAGE)
    [q, qd, stop_condition] = two_link_left_masses(q(1:2,end), qd(1:2), parameters, params);
    
    % check whether it's still walking
    if(stop_condition)
        break;
    end
    
    %Save LEFT step length to array for each step
    left_step_length(steps) = abs(LL*sin(q(1,end)) - LR*sin(q(2,end)));
    
	
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    Right Stance    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	
    % Calls dynamics of two link RIGHT 
	% (RIGHT LEG IS ON THE GROUND, MECHANISM IS IN TWO LINK STAGE)
    [q2, qd, stop_condition] = two_link_right_masses([q(2,end); q(1,end)], [qd(1); -qd(2)], parameters, params);

    % check whether it's still walking
    if(stop_condition)
        break;
    end
    
    %Save LEFT step length to array for each step
    right_step_length(steps) = abs(LR*sin(q2(1,end)) - LL*sin(q2(2,end)));
	
	%-------------------------------------------------------------------------------------
    
    % Switch the position references for each limb
	% NOTE: "2" refers to post heel strike
    q(1) = q2(2); 
    q(2) = q2(1);
    q(3) = q2(1); 
    
    % Assign velocities each limb after heel strike
    qd = [qd(1) -qd(2) -qd(2)]';  
    
	%-------------------------------------------------------------------------------------
	
    % Check if this entire previous step was successful
    if(steps == total_steps + 1)
        successful = true; 
	end
end


% Display walker limit cycle plot -  This is plotted inside the dynamics functions
if (cycle)
    figure(127)
    set(get(gcf,'CurrentAxes'),'FontName','Times New Roman','FontSize',20)
    xlabel('Angle (rads)')
    ylabel('Angle (rads/s)')
    % title(strcat(['Limit Cycle Trajectory Plot ' num2str(changed_variable)]))
    set(gcf, 'PaperPosition', [1 1 4.2 4.0]);
	grid on
    
    % Save plot to folder
    print('-dpdf','-r300',strcat([data_path 'map_' num2str(changed_variable) '.pdf']))
    hold off
end

% Display walker step Length Plot
if (step_length)
    figure(123)
    clf
    plot(left_step_length,'*b')
    hold on
    plot(right_step_length,'*g')
    set(get(gcf,'CurrentAxes'),'FontName','Times New Roman','FontSize',20)
    xlabel('Step Number')
    ylabel('Step Length (meters)')
    % title(strcat(['Step Length Plot ' num2str(changed_variable) ' 0.26']))
    legend('Left leg step length','Right leg step length')
    set(gcf, 'PaperPosition', [1 1 4.2 4.0]);
	grid on
    
    % Save plot to folder
    print('-dpdf','-r300',strcat([data_path 'step_' num2str(changed_variable) '.pdf']))
    hold off
end

end
