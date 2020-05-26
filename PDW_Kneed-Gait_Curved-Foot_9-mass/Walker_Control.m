%  Full Model
%  This model describes a 3-Link passive dynamic walker in terms of each
%  individual legs
function [stop_condition, steps, successful, right_step_length, left_step_length, right_step_time, left_step_time, EnergyStat, ForcesStat] ...
    = Walker_Control(q, qd, parameters, params, total_steps, FootData)

%Define all parameters by assigning them indecies of the incoming
%"parameters" 
mt1R = parameters(1);      mt1L = parameters(14); 
mt2R = parameters(2);      mt2L = parameters(15); 
ms2R = parameters(3);      ms2L = parameters(16);
ms1R = parameters(4);      ms1L = parameters(17);
a1R  = parameters(5);      a1L  = parameters(18);
b1R  = parameters(6);      b1L  = parameters(19); 
c1R  = parameters(7);      c1L  = parameters(20);
c2R  = parameters(8);      c2L  = parameters(21);
b2R  = parameters(9);      b2L  = parameters(22);
a2R  = parameters(10);     a2L  = parameters(23);  
lsR  = parameters(11);     lsL  = parameters(24);
ltR  = parameters(12);     ltL  = parameters(25);
LR   = parameters(13);     LL   = parameters(26);

mh   = parameters(27);

g     = parameters(28);     
dt    = parameters(29);    
theta = parameters(30);

time_plot           = parameters(31);       changed_variable = parameters(42);     
total_steps         = parameters(32);       velocity_disp    = parameters(43);
display             = parameters(33);       force            = parameters(44);   
energy              = parameters(34);       energy_cycle     = parameters(45);
step_length         = parameters(35);       angle            = parameters(46);   
limit_cycle         = parameters(36);       angular_vel      = parameters(47);
save_walking_figure = parameters(37);       step_time        = parameters(48);
print_count         = parameters(38);
mhmul               = parameters(39); 
mtmul               = parameters(40);         
msmul               = parameters(41);
   
%For animation saving into data_path directory
data_name = params.data_name;       
other     = params.other;
data_path = strcat([data_name '/']);

start_time = time_plot;
stop_condition = 0;     %Once it becomes 1 code will stop (opposite of successful variable)
successful = false;     %Walker has to walk total number of steps, is going to be true if walker completes all total steps

rLb = FootData(1, 2);
rRb = FootData(2, 2);

realtime(1) = 0;  %Global time
     Energy = []; %Energy data
 EnergyStat = []; 
     Forces = []; %Kinetic Data
 ForcesStat = [];
     Angles = []; %leg angles
    Ang_Vel = []; %leg velocites

%Creating blank matrix
 left_step_length = [];       right_step_length = [];
 left_step_time   = [];         right_step_time   = [];
    left_YGRF_max = [];          right_YGRF_max = [];


for steps = 1:total_steps;   
    
    StepTimeKeeper_1 = realtime(end);  %keeps time of step
    
    %Calls dynamics of THREE link LEFT (LEFT foot is on ground)
    [q, qdp, stop_condition, realtime, Energy, EnergyStat, Forces, ForcesStat, Angles, Ang_Vel] ...
        = Stance_LEFT_3L(q, qd, parameters, params, steps, ...
          realtime, Energy, EnergyStat, Forces, ForcesStat, FootData, Angles, Ang_Vel);
    count = length(q);
   
    %Check whether walker is still walking
    if(stop_condition); break; end 

    %Calls dynamics of TWO link LEFT (LEFT foot is on ground)
    [q, qdp, stop_condition, realtime, Energy, EnergyStat, Forces, ForcesStat, Angles, Ang_Vel] ...
        = Stance_LEFT_2L([q(1);q(2)], [qdp(1);qdp(2)], parameters, params, steps, ...
          realtime, Energy, EnergyStat, Forces, ForcesStat, FootData, Angles, Ang_Vel);
    count = count + length(q);
      
    %Check whether walker is still walking
    if(stop_condition); break; end 
    
    %Kinetic Data (maximum)
    if(force)
        ForcesStat = [ForcesStat; [max(Forces(length(Forces)-count:end, 2)), max(Forces(length(Forces)-count:end, 3)) ] ];
    end
    
    %LEFT step length   
    rL = FootData(1,1) + FootData(1,2)*q(1, end);
    rR = FootData(2,1) + FootData(2,2)*q(2, end);
    
    [psi, psi2, d_a, phi, theta_a, d_a2, phi2, theta_a2, height] ...
        = gContact('L', FootData, theta, q(1,end), q(2,end), rL, rR, rLb, rRb, LL, LR);

    left_step_length(steps) = abs( (LL*sin(q(1,end) + d_a*sin(pi - (pi/2-q(1,end)) - theta_a) ))...
                            - (LR*sin(q(2,end) + d_a2*sin(pi - (pi/2-q(2,end)) - theta_a2) )) );
    
    %LEFT step time
    StepTimeKeeper_2 = realtime(end);
    left_step_time(steps) = StepTimeKeeper_2 - StepTimeKeeper_1;
    
    
    %================ SWITCH STANCE FOOT =============================
    
    
    StepTimeKeeper_1 = realtime(end);
    
    %Calls dynamics of THREE link RIGHT (Right foot is on ground)
    [q, qdp, stop_condition, realtime, Energy, EnergyStat, Forces, ForcesStat, Angles, Ang_Vel] ...
        = Stance_RIGHT_3L([q(2,end); q(1,end); (q(1,end)-.01)],[qdp(1) -qdp(2) -qdp(2)]' ,...
          parameters, params, steps, realtime, Energy, EnergyStat,...
          Forces, ForcesStat, FootData, Angles, Ang_Vel);
    count = length(q);
    
    %Check whether walker is still walking
    if(stop_condition); break; end 
    
    
    %Calls dynamics of two link RIGHT (Right foot is on ground)
    [q2, qdp, stop_condition, realtime, Energy, EnergyStat, Forces, ForcesStat, Angles, Ang_Vel] ...
        = Stance_RIGHT_2L([q(1);q(2)], [qdp(1);qdp(2)],...
          parameters, params, steps, realtime, Energy, EnergyStat,...
          Forces, ForcesStat, FootData, Angles, Ang_Vel);
    count = count + length(q2);

    %Check whether walker is still walking
    if(stop_condition); break; end 
    
	%Kinetic Data (maximum)
    if(force)
        ForcesStat = [ForcesStat; [max(Forces(length(Forces)-count:end, 2)), max(Forces(length(Forces)-count:end, 3)) ] ];
    end
    
    %RIGHT step length   
    rR = FootData(2,1) + FootData(2,2)*q2(1, end);
    rL = FootData(1,1) + FootData(1,2)*q2(2, end);
    
    [psi, psi2, d_a, phi, theta_a, d_a2, phi2, theta_a2, height] ...
        = gContact('R', FootData, theta, q2(1,end), q2(2,end), rR, rL, rRb, rLb, LR, LL);

    right_step_length(steps) = abs( (LR*sin(q2(1,end) + d_a*sin(pi - (pi/2-q2(1,end)) - theta_a) ))...
                            - (LL*sin(q2(2,end) + d_a2*sin(pi - (pi/2-q2(2,end)) - theta_a2) )) );
    
    %RIGHT step time
    StepTimeKeeper_2 = realtime(end);
    right_step_time(steps) = StepTimeKeeper_2 - StepTimeKeeper_1;
    
          
    
    %Angles
    q(1) = q2(2);        %switch angle definition
    q(2) = q2(1);        %switch angle definition
    q(3) = q2(1) - .01;  % adjust this as needed, add slight bend to knee
     
    %Velocities
    qd = [qdp(1), -qdp(2), -qdp(2)]';  %qdp = velocities post heel strike
    
    
    
    %Increase Step Number
    steps = steps + 1; 
       
    %Find if walker completed total steps
    if(steps == total_steps + 1)
        successful = true;   
    end
end






%Energy Plot
if(energy)  && (steps > 0)
    figure(6)
    plot(Energy(1,:), Energy(2,:),'-k','LineWidth',4)
    hold on
    plot(Energy(1,:), Energy(3,:), '-r','LineWidth',3)
    plot(Energy(1,:), Energy(4,:), '-b','LineWidth',3)

    xlabel('Time (Seconds)')
    ylabel('Energy (Joules)')
    title('PDW Energy','FontName','Times New Roman','FontSize',20)
    legend('Total Energy', 'Potential Energy', 'Kinetic Energy')
    set(get(gcf,'CurrentAxes'),'FontName','Times New Roman','FontSize',15)
    grid on
    
    if length(EnergyStat) > 14
        fprintf('Avg. Work/Step Left  Leg: %1.3f J\n',   mean(EnergyStat(13:2:end,1)))
        fprintf('Avg. Work/Step Right Leg: %1.3f J\n',   mean(EnergyStat(14:2:end,1)))
        fprintf('Avg. Work/Step Both Legs: %1.3f J\n\n', mean(EnergyStat(13:1:end,1)))
    end
end



%Kinetic Plot
if(force)  && (steps > 0)
    figure(7)
    plot(Forces(:,1), Forces(:,2), 'r', 'LineWidth',3)
    hold on
    plot(Forces(:,1), Forces(:,3), 'b', 'LineWidth',3)
    legend('Rx (N)','Ry (N)')
    xlabel('Time (Seconds)')
    ylabel('Force (Newtons)')
    title('PDW Ground Reaction Forces','FontName','Times New Roman','FontSize',20)
    set(get(gcf,'CurrentAxes'),'FontName','Times New Roman','FontSize',15)
    grid on
    
    begin = (round(0.20*length(ForcesStat))); %start from 20% of data
    begin = 2*round((begin+1)/2)-1;           %odd number
    if length(ForcesStat) >= begin
        fprintf('Max Left  X-GRF: %1.3f N\n',   mean(ForcesStat(begin:2:end,1)))
        fprintf('Max Right X-GRF: %1.3f N\n',   mean(ForcesStat(begin+1:2:end,1)))
        Symmetry_Fx = abs(( mean(ForcesStat(begin:2:end,1))/mean(ForcesStat(begin+1:2:end,1)) ) - 1) * 100;
        fprintf('X-GRF Asymmetry: %1.3f %%\n\n', Symmetry_Fx)
        
        fprintf('Max Left  Y-GRF: %1.3f N\n',   mean(ForcesStat(begin:2:end,2)))
        fprintf('Max Right Y-GRF: %1.3f N\n', mean(ForcesStat(begin+1:2:end,2)))
        Symmetry_Fy = abs(( mean(ForcesStat(begin:2:end,2))/mean(ForcesStat(begin+1:2:end,2)) ) - 1) * 100;
        fprintf('Y-GRF Asymmetry: %1.3f %%\n\n', Symmetry_Fy)
    end
end



%Limit limit_cycle plot -  This is plotted inside the dynamics functions
if (limit_cycle)  && (steps > 2)
    figure(8);
    xlabel('Angle (Radians)')
    ylabel('Angle (Radians/Second)')
    title(strcat(['Limit Cycle Trajectory Plot ' num2str(changed_variable)]),'FontName','Times New Roman','FontSize',20)
    set(gcf, 'PaperPosition', [1 1 4.2 4.0]);
    set(get(gcf,'CurrentAxes'),'FontName','Times New Roman','FontSize',15)
    grid on
    hold off
end



%Step Length Plot
if (step_length) && (steps > 3)
    figure(9);
    clf
    plot(left_step_length, 'ob', 'markersize', 7,'MarkerFaceColor','b')
    hold on
    plot(right_step_length, 'sr', 'markersize', 6, 'MarkerFaceColor','r')
    xlabel('Step Number')
    ylabel('Step Length (Meters)')
	title('Step Length','FontName','Times New Roman','FontSize',20)
    legend('Left Step','Right Step')
    set(get(gcf,'CurrentAxes'),'FontName','Times New Roman','FontSize',15);
    set(gcf, 'PaperPosition', [1 1 4.2 4.0]);
    axis([1, length(left_step_length),  0, mean((left_step_length + right_step_length)/2)*1.75])
    set(gcf,'position', [9   576   743   420])
    %axis auto
    grid on
    hold off
    
    begin = round(0.60*length(left_step_length)); %start from 60% of data
    fprintf('\nAverage Left  Step Length: %1.3f m\n',mean(left_step_length(begin:end)))
    fprintf('Average Right Step Length: %1.3f m\n',mean(right_step_length(begin:end)))
    Symmetry_length = (abs((mean(left_step_length(begin:end))/mean(right_step_length(begin:end)))-1)) * 100;
    fprintf('Step Length Asymmetry:     %1.3f %%\n', Symmetry_length)
end



%Step Time Plot
if (step_time) && (steps > 3)
    figure(10);
    clf
    plot(left_step_time, 'ob', 'markersize', 7,'MarkerFaceColor','b')
    hold on
    plot(right_step_time, 'sr', 'markersize', 6, 'MarkerFaceColor','r')
    xlabel('Step Number')
    ylabel('Step Time (Seconds)')
	title('Step Time','FontName','Times New Roman','FontSize',20)
    legend('Left Step','Right Step')
    set(get(gcf,'CurrentAxes'),'FontName','Times New Roman','FontSize',15);
    set(gcf, 'PaperPosition', [1 1 4.2 4.0]);
    axis([1, length(left_step_time),  0, mean((left_step_time + right_step_time)/2)*1.75])
    set(gcf,'position', [768   575   743   420])
    %axis auto
    grid on
    hold off
    
    begin = round(0.60*length(left_step_time)); %start from 60% of data
    fprintf('\nAverage Left  Step Time: %1.3f s\n',mean(left_step_time(begin:end)))
    fprintf('Average Right Step Time: %1.3f s\n',  mean(right_step_time(begin:end)))
    Asymmetry_time = (abs((mean(left_step_time(begin:end))/mean(right_step_time(begin:end)))-1)) * 100;
    fprintf('Step Time Asymmetry:     %1.3f %%\n', Asymmetry_time)
end




%Leg and Hip Angle Plot
if (angle) && (steps > 2)
    h6 = figure(11);
    set(0, 'CurrentFigure', h6)
    plot(Angles(:,1), Angles(:,2), '-r', 'LineWidth',3)
    hold on
    plot(Angles(:,1), Angles(:,3), '-b', 'LineWidth',3)
    plot(Angles(:,1), (Angles(:,2) + Angles(:,3)), '-k', 'LineWidth',3)
    grid on
    title('Leg and Hip Angle','FontName','Times New Roman','FontSize',20)
    xlabel('Time (Seconds)')
    ylabel('Angle (Radians)')
    legend('Left Leg', 'Right Leg', 'Hip Angle')
    set(get(gcf,'CurrentAxes'),'FontName','Times New Roman','FontSize',15);
end



%Leg and Hip Angular Velocity
if (angular_vel) && (steps > 2)
    h5 = figure(130);
    set(0, 'CurrentFigure', h5)
    plot(Ang_Vel(:,1), Ang_Vel(:,2), '-r', 'LineWidth',3)
    hold on
    plot(Ang_Vel(:,1), Ang_Vel(:,3), '-b', 'LineWidth',3)
    grid on
    title('Leg and Hip Angular Velocity','FontName','Times New Roman','FontSize',20)
    xlabel('Time (Seconds)')
    ylabel('Angular Velocity (Radians/Second)')
    legend('Left Leg', 'Right Leg')
    set(get(gcf,'CurrentAxes'),'FontName','Times New Roman','FontSize',15);
end




end
