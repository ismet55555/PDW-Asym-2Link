function plot_results(p, results)
%UNTITLED8 
%   Summary of this function goes here
%   Detailed explanation goes here



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    Angular Position    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Cummalative / All
if (p.sim.output.angle)
    % Setting the window name
    p.sim.figure_handles.q_time = figure('Name' ,"PDW: Angular Position", ...
                                         'NumberTitle','off');
    % Figure background color
    set(gcf, 'color', 'w');
    hold on

    % Left leg
    plot(results.sim.time, results.motion.all.left.q.data, '-r', 'LineWidth', 1)
    
    % Right leg
    plot(results.sim.time, results.motion.all.right.q.data, '-b', 'LineWidth', 1)
    
    legend('Left Leg', 'Right Leg')
    title('Angular Position of Legs Over Time')
    xlabel('Time (s)')
    ylabel('Angular Position (rad)')

    set(get(gcf,'CurrentAxes'), 'FontName', 'Times New Roman', 'FontSize', 12);
    grid on
    box on
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    Angular Velocity    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if (p.sim.output.angular_vel)
    p.sim.figure_handles.qd_time = figure('Name' ,"PDW: Angular Velocity", ...
                                          'NumberTitle','off');
    set(gcf, 'color', 'w');
    hold on
    
    plot(results.sim.time, results.motion.all.left.qd.data,  '-r', 'LineWidth', 1)
    plot(results.sim.time, results.motion.all.right.qd.data, '-b', 'LineWidth', 1)
    
    legend('Left Leg', 'Right Leg') 
    title('Angular Velocity of Legs Over Time')      
    xlabel('Time (s)')
    ylabel('Angular Velocity (rad/s)')    
    
    set(get(gcf,'CurrentAxes'), 'FontName', 'Times New Roman', 'FontSize', 12);
    grid on
    box on
end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    Angular Acceleration    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if (p.sim.output.angular_accel)
    p.sim.figure_handles.qdd_time = figure('Name' ,"PDW: Angular Acceleration", ...
                                           'NumberTitle','off');
    set(gcf, 'color', 'w');
    hold on
    
    plot(results.sim.time, results.motion.all.left.qdd.data,  '-r', 'LineWidth', 1)
    plot(results.sim.time, results.motion.all.right.qdd.data, '-b', 'LineWidth', 1)
    
    legend('Left Leg', 'Right Leg') 
    title('Angular Acceleration of Legs Over Time')   
    xlabel('Time (s)')
    ylabel('Angular Acceleration (rad/s^2)')    
    
    set(get(gcf,'CurrentAxes'), 'FontName', 'Times New Roman', 'FontSize', 12);
    grid on
    box on
end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    Energy    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if(p.sim.output.energy)
    % PDW Energy Over Time
    p.sim.figure_handles.energy_all = figure('Name' ,"PDW: Energy Over Time", ...
                                              'NumberTitle','off');
    set(gcf, 'color', 'w');
    hold on
    
    plot(results.sim.time, results.energy.all.total.data,'-k','LineWidth', 1)
    plot(results.sim.time, results.energy.all.PE.data,   '-m','LineWidth', 1)
    plot(results.sim.time, results.energy.all.KE.data,   '-b','LineWidth', 1)
    
    xlabel('Time (s)')
    ylabel('Energy (J)')
    title('PDW Energy Over Time')
    legend('Total Energy', 'Potential Energy', 'Kinetic Energy')
    set(get(gcf,'CurrentAxes'), 'FontName', 'Times New Roman', 'FontSize', 12);
    grid on
    box on
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % PDW Energy Per Step
    p.sim.figure_handles.energy_per_step = figure('Name' ,"PDW: Energy Per Step", ...
                                              'NumberTitle','off');
    set(gcf, 'color', 'w');
    hold on
    
    
    for i = 1 : 2 : length(results.sim.steps)
        x = results.energy.left_right.total.time{i};
        y = results.energy.left_right.total.data{i};
        plt = plot(x, y, '-r','LineWidth', 1);
        plt.Color(4) = 0.4;      
        
        x = results.energy.left_right.PE.time{i};
        y = results.energy.left_right.PE.data{i};
        plt = plot(x, y, '-r','LineWidth', 1);
        plt.Color(4) = 0.3;        
        
        x = results.energy.left_right.KE.time{i};
        y = results.energy.left_right.KE.data{i};
        plt = plot(x, y, '-r','LineWidth', 1);
        plt.Color(4) = 0.3;
    end
    

    for i = 2 : 2 : length(results.sim.steps)
        x = results.energy.left_right.total.time{i};
        y = results.energy.left_right.total.data{i};
        plt = plot(x, y, '-b','LineWidth', 1);
        plt.Color(4) = 0.4;      
        
        x = results.energy.left_right.PE.time{i};
        y = results.energy.left_right.PE.data{i};
        plt = plot(x, y, '-b','LineWidth', 1);
        plt.Color(4) = 0.3;        
        
        x = results.energy.left_right.KE.time{i};
        y = results.energy.left_right.KE.data{i};
        plt = plot(x, y, '-b','LineWidth', 1);
        plt.Color(4) = 0.3;
    end
    
    xlabel('Time (s)')
    ylabel('Energy (J)')
    title('PDW Energy Per Step')
%     legend('Total Energy', 'Potential Energy', 'Kinetic Energy')
    set(get(gcf,'CurrentAxes'), 'FontName', 'Times New Roman', 'FontSize', 12);
    grid on
    box on
end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    Forces    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if(p.sim.output.force)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % PDW Forces Over Time
    p.sim.figure_handles.forces_time = figure('Name' ,"PDW: Ground Forces Over Time", ...
                                              'NumberTitle','off');
    set(gcf, 'color', 'w');
    hold on
    
    plot(results.sim.time, results.force.all.Rx.data, 'm', 'LineWidth', 1)
    plot(results.sim.time, results.force.all.Ry.data, 'b', 'LineWidth', 1)
    
    legend('Rx','Ry')
    title('PDW Ground Reaction Forces Over Time')    
    xlabel('Time (s)')
    ylabel('Ground Force (N)')
    
    set(get(gcf,'CurrentAxes'), 'FontName', 'Times New Roman', 'FontSize', 12);
    grid on
    box on
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % PDW Forces Per Step
    p.sim.figure_handles.forces_time = figure('Name' ,"PDW: Ground Forces Per Step", ...
                                              'NumberTitle','off');
    set(gcf, 'color', 'w');
    hold on
    
    % Left Stance
    for i = 1 : 2 :length(results.sim.steps)
        x = results.force.left.Rx.time{i};
        y = results.force.left.Rx.data{i};
        plt = plot(x, y, '-r','LineWidth', 1);
        plt.Color(4) = 0.3;  
        
        x = results.force.left.Ry.time{i};
        y = results.force.left.Ry.data{i};
        plt = plot(x, y, '-r','LineWidth', 1);
        plt.Color(4) = 0.3;  
    end
    
    % Right Stance
    for i = 2 : 2 :length(results.sim.steps)
        x = results.force.right.Rx.time{i};
        y = results.force.right.Rx.data{i};
        plt = plot(x, y, '-b','LineWidth', 1);
        plt.Color(4) = 0.3;  
        
        x = results.force.right.Ry.time{i};
        y = results.force.right.Ry.data{i};
        plt = plot(x, y, '-b','LineWidth', 1);
        plt.Color(4) = 0.3;  
    end
    
    title('PDW Ground Reaction Forces Per Step')    
    xlabel('Time (s)')
    ylabel('Ground Force (N)')
    
    set(get(gcf,'CurrentAxes'), 'FontName', 'Times New Roman', 'FontSize', 12);
    grid on
    box on
end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    Step Length    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if (p.sim.output.step_length)
    p.sim.figure_handles.step_length = figure('Name' ,"PDW: Step Length Per Step", ...
                                              'NumberTitle','off');
    set(gcf, 'color', 'w');
    hold on

    plot(results.sim.strides,  results.step_length.left, '--r', 'LineWidth', 1)
    plot(results.sim.strides, results.step_length.right, '--b', 'LineWidth', 1)

    plot(results.sim.strides, results.step_length.left, ...
        'or', 'markersize', 4, 'MarkerFaceColor', 'r');
    plot(results.sim.strides, results.step_length.right, ...
        'ob', 'markersize', 3, 'MarkerFaceColor', 'b');

    legend('Left Leg', 'Right Leg')
    title('PDW Step Length Per Step')
    xlabel('Step Number')
    ylabel('Step Length (m)')

    set(get(gcf,'CurrentAxes'), 'FontName', 'Times New Roman', 'FontSize', 12);
    ylim([0, max([results.step_length.left, results.step_length.right])*1.25])
    grid on
    box on
    
    % Make sure there is not non-integer x-axis tick values
    if results.sim.stride < 10
        set(gca,'xtick', results.sim.strides)
    else
end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    Limit Cycle    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if (p.sim.output.limit_cycle)
    % Setting the window name
    p.sim.figure_handles.q_time = figure('Name' ,"PDW: Limit Cycle", ...
                                         'NumberTitle','off');
    % Figure background color
    set(gcf, 'color', 'w');
    hold on

    % Left leg
    plt = plot(results.motion.all.left.q.data, results.motion.all.left.qd.data, '-r', 'LineWidth', 1);
    plt.Color(4) = 0.2;
    
    % Right leg
    plt = plot(results.motion.all.right.q.data, results.motion.all.right.qd.data, '-b', 'LineWidth', 1);
    plt.Color(4) = 0.2;
    
    legend('Left Leg', 'Right Leg')
    title('Limit Cycle of Legs Over Time')
    xlabel('Angular Position (rad)')
    ylabel('Angular Velocity (rad/s)')   

    set(get(gcf,'CurrentAxes'), 'FontName', 'Times New Roman', 'FontSize', 12);
    grid on
    box on
end






end

