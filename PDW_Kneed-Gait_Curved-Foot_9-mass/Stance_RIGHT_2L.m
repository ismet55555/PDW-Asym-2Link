%Right Stance (Right foot is on ground)

function [q, qdp, stop_condition, realtime, Energy, EnergyStat, Forces, ForcesStat, Angles, Ang_Vel] ...
                = Stance_RIGHT_2L(q,qd,parameters,params, steps, realtime, ...
                  Energy, EnergyStat, Forces, ForcesStat, FootData, Angles, Ang_Vel)

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
save_walking_figure = parameters(37);     
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


%Define foot radii:
rRa = FootData(2,1);    rRb = FootData(2,2);   dR = FootData(2,3);
rR = zeros(1, 10000);   rR(1) = rRa + rRb*q(2, end);

rLa = FootData(1,1);     rLb = FootData(1,2);   dL = FootData(1,3);
rL = zeros(1, 10000);    rL(1) = rLa + rLb*q(1, end);

%========== Creating Shoe Rim (Curve) ==============
qf = [linspace(pi/2, -pi/2, 15); linspace(pi/2, -pi/2, 15)]; 
rRf = ones(1,length(qf))*rRa + rRb*qf(1,:);  %stance (right) foot
rLf = ones(1,length(qf))*rLa + rLb*qf(2,:);  %swing (left) foot
rim_dat = zeros(length(qf), 4);
for i = 1:length(qf)
   [psi, psi2, d_a, phi, theta_a, d_a2, phi2, theta_a2, height] ...
       = gContact('R', FootData, theta, qf(1,i), qf(2,i), rRf(i), rLf(i), 0, 0, LR, LL);
   
   rim_dat(i,:) = [d_a, theta_a, d_a2, theta_a2];
end
%===================================================


%Finding initial hip mass height:
[psi, psi2, d_a, phi, theta_a, d_a2, phi2, theta_a2, height] ...
    = gContact('R', FootData, theta, q(1,end), q(2,end), rR(1), rL(1), rRb, rLb, LR, LL);


dt_dumdum = dt;
ramp = -100;
count = 2;
time(1) = 0;
while(  (   height -  LL*cos(q(2,count-1)) - d_a2*sin( theta_a2 - q(2,count-1))    ) > ramp   )    
    lengthLR = LR*sin(q(1,count-1)) + d_a *cos( theta_a -  q(1,count-1));
    lengthLL = LL*sin(q(2,count-1)) + d_a2*cos( theta_a2 - q(2,count-1));
    ramp = -(-lengthLR + lengthLL)*tan(theta);
    %Clearing ramp 
    if qd(2,count-1) > 0;        
        ramp = ramp - 0.4; 
        dt = dt_dumdum + 0.002;
        dt = dt_dumdum;
    else
        dt = dt_dumdum;
    end
    
    q1 = q(1, count - 1);       q1d = qd(1, count - 1);
    q2 = q(2, count - 1);       q2d = qd(2, count - 1);

    %Calling the right stance dynamics function
    [qdd(:,count)] = Dynamics_2L_RIGHT(q1, q1d, q2, q2d, rLa, rLb, dL, rRa, rRb, dR, parameters, psi);
    
    %Angular Velocities (Numerical Integration)
    qd(:,count) = qd(:,count-1) + qdd(:,count)*dt;
    %Angular Positions (Numerical Integration)
    q(:,count) = q(:,count-1) + qd(:,count)*dt;
    
    %======================================================================
    %======================================================================
    
    
    %Time step
    time(count) = time(count-1) + dt;  
    
    %Define Foot Radius:
    %Right Foot
    rR(count) = rRa + rRb*q(1, count);
    %Left Foot
    rL(count) = rLa + rLb*q(2, count);

    
    %========================================================
    %Energy Data
    if(energy)
        [E_cont(count), KE(count), PE(count)] ...
                    = Energy_RIGHT_2L(q(:,end), qd(:,end),...
                      parameters, FootData, rR(count));
    end
    %========================================================
    
    
    %========================================================
    %Kinetic Data
    if (force)
        Rx = 0;   Ry = 0;
        [Rx, Ry] = Forces_RIGHT_2L(q(:,end), qd(:,end), qdd(:,end), parameters, FootData, rR(count));
        Forces = [Forces; time(count)+realtime(end), Rx, Ry];
    end
    %========================================================  
    
    
    %=================== Collision Detection ================    
    [psi, psi2, d_a, phi, theta_a, d_a2, phi2, theta_a2, height] ...
        = gContact('R', FootData, theta, q(1,count), q(2,count), rR(count), rL(count), rRb, rLb, LR, LL);
    %======================================================== 
    
    
    %Increase counter
    count = count + 1;
    
    
    %=================== Model Failure Diagnosis ============
    %If the loop exceeds iteration threshhold then stop
    TooLong = 0;
    FellForward = 0;
    FellBackward = 0;
    
    if(count > 25000)
        TooLong = 1;
        stop_condition = 1;
        break;
    end
    if (q(1,count-1)+theta < -pi/2)
        FellForward = 1;
        %stop_condition = 1;
        break;
    end
    if(q(1,count-1)+theta > pi/2)
        FellBackward = 1;
        %stop_condition = 1;
        break;
    end
    
end
dt = dt_dumdum;
  

%Energy data
%EnergyStat = [EnergyStat ; PE(2) - PE(end)];
if(energy) && (stop_condition == 0)   
    E_cont(1) = [];   PE(1) = [];    KE(1) = [];
    time_e = time + realtime(end);
    time_e(1) = [];
    Energy = [Energy, [time_e; E_cont; PE; KE] ];
end


%Energy Cycle Plot
if(energy_cycle) && (stop_condition == 0)%  && (steps == 8)
    h4 = figure(128);
    set(0, 'CurrentFigure', h4)
    plot(PE, KE, '-b', 'linewidth', 1)
    hold on
    grid on
    title('Energy Cycle')
	xlabel('Potential Energy (J)')
	ylabel('Kinetic Energy (J)')
end


%Limit Cycle
if (limit_cycle) && (stop_condition == 0) 
    h2 = figure(127);
    set(0, 'CurrentFigure', h2)
    if (display)
        plot(q(1,:),qd(1,:),':b')
    else
        plot(q(1,:),qd(1,:),'-b')
    end
    hold on
    plot(q(1,1),qd(1,1),'*m')
end

%Angle Progression Plot
if (angle) && (stop_condition == 0)%  && (steps > 9)
   Angles = [Angles; time'+realtime(end),  q(2,:)', q(1,:)'];
end

%Angular Velocity Plot
if (angular_vel) && (stop_condition == 0)% && (steps > 9)
   Ang_Vel = [Ang_Vel; time'+realtime(end),  qd(2,:)', qd(1,:)'];
end


realtime = [realtime,  realtime(end) + time];
time_plot = time_plot + count;
stop_time = start_time + count-3;

%Deleting zeros in radius vector
rR(rR == 0) = [];
rL(rL == 0) = [];



%---------------------- Simulation Animation ------------------------------
if(length(q) < 20) || (stop_condition == 1)
    
    %Stop if ran less then threshhold
    fprintf('\n=======================================')
    fprintf('\n=========== MODEL FAILED! =============')
    fprintf('\n=======================================')
    if (TooLong == 1)
      fprintf('\n  Model ran %g Iterations', length(q))  
    end
    if (FellForward == 1)
      fprintf('\n Fell forward in stride %g, right step', steps)  
    end
    if (FellBackward == 1)
      fprintf('\n Fell backward in stride %g, right step', steps)   
    end
    fprintf('\n=======================================\n\n')
        
    stop_condition = 1;
    
elseif(display)
    
    h = figure(999);
    set(h,'DoubleBuffer','on');
    set(h,'position',[-871     6   863   677]);
    
    n = 20; %Animation step
    for loop = [1:n:length(q)-1, length(q)-1]
        
        [psi, psi2, d_a, phi, theta_a, d_a2, phi2, theta_a2, height] ...
            = gContact('R', FootData, theta, q(1,loop), q(2,loop), rR(loop), rL(loop), rRb, rLb, LR, LL);

        %----------------------

        %Drawing the ramp
        set(0, 'CurrentFigure', h)
        plot([LR*sin(q(1,loop))- 3*cos(theta) + d_a*sin(pi-(pi/2 - q(1,loop))-theta_a), LR*sin(q(1,loop))+3*cos(theta)+ d_a*sin(pi-(pi/2 - q(1,loop))-theta_a)], ...
            height+[-LR*cos(q(1,loop)) + 3*sin(theta) - d_a*cos(pi-(pi/2 - q(1,loop))-theta_a), -LR*cos(q(1,loop))-3*sin(theta)-d_a*cos(pi-(pi/2 - q(1,loop))-theta_a) ],...
            '-k','LineWidth',5)
        hold on
        p = patch([LR*sin(q(1,loop))- 3*cos(theta) + d_a*sin(pi-(pi/2 - q(1,loop))-theta_a), LR*sin(q(1,loop))+3*cos(theta)+ d_a*sin(pi-(pi/2 - q(1,loop))-theta_a), 10, -10],...
        height+[-LR*cos(q(1,loop)) + 3*sin(theta) - d_a*cos(pi-(pi/2 - q(1,loop))-theta_a), -LR*cos(q(1,loop))-3*sin(theta)-d_a*cos(pi-(pi/2 - q(1,loop))-theta_a), -5,-5 ],...
        [0.5,0.7,0.5] );
        
        axis([-1.75 1.75 -0.5 2.5]);
        box on
        %grid on
        
        %=== Link 1 ===
        plot([0, LR*sin(q(1,loop))], height+[0, -LR*cos(q(1,loop))],'color',[.2 .6 .8],'LineWidth',6)
        %Plot link 1 base to ground contact:
        plot([(LR*sin(q(1,loop)))+ d_a*sin(pi-(pi/2 - q(1,loop))-theta_a), LL*sin(q(1,loop))], ...
             [height-(LR*cos(q(1,loop))+d_a*cos(pi-(pi/2 - q(1,loop))-theta_a)), (height-LL*cos(q(1,loop)))] ,':b','LineWidth',2,'color',[.2 .6 .8])
        %Plot curve top for stance  
        plot([LR*sin(q(1,loop))-(rRf(1)-dR)*sin(pi/2-q(1,loop)) , LR*sin(q(1,loop))+(dR+rRf(end))*sin(pi/2-q(1,loop))],...
            [height-LR*cos(q(1,loop))-(rRf(1)-dR)*cos(pi/2-q(1,loop)) , height-LR*cos(q(1,loop))+(dR+rRf(end))*cos(pi/2-q(1,loop))],...
            '-k','LineWidth',4) 
        %right foot contact:  
        plot((LR*sin(q(1,loop)))+ d_a*sin(pi-(pi/2 - q(1,loop))-theta_a), ...
             height-(LR*cos(q(1,loop))+d_a*cos(pi-(pi/2 - q(1,loop))-theta_a)), 'color',[.2 .6 .8], 'markersize', 5, 'MarkerFaceColor', [.2 .6 .8])

        %----------------------

        %=== Link 2 ===:
        plot([0, LL*sin(q(2,loop))],height+[0, -LL*cos(q(2,loop))],'color',[.8 .6 .6],'LineWidth',6)
        %Plot link 2 base to shoe base
        plot([LL*sin(q(2,loop)), (LL*sin(q(2,loop)))+ d_a2*sin(pi-(pi/2 - q(2,loop))-theta_a2)],...
                height+[(-LL*cos(q(2,loop))), -(LL*cos(q(2,loop))+d_a2*cos(pi-(pi/2 - q(2,loop))-theta_a2))],':r','LineWidth',2,'color',[.8 .6 .6])
        %Plot curve top for swing
        plot([LL*sin(q(2,loop))-(rLf(1)-dL)*sin(pi/2-q(2,loop)) , LL*sin(q(2,loop))+(dL+rLf(end))*sin(pi/2-q(2,loop))],...
            [height-LL*cos(q(2,loop))-(rLf(1)-dL)*cos(pi/2-q(2,loop)) , height-LL*cos(q(2,loop))+(dL+rLf(end))*cos(pi/2-q(2,loop))],...
            '-k','LineWidth',4)
        %left foot contact:                 
        plot((LL*sin(q(2,loop)))+ d_a2*sin(pi-(pi/2 - q(2,loop))-theta_a2), ...
             height-(LL*cos(q(2,loop))+d_a2*cos(pi-(pi/2 - q(2,loop))-theta_a2)), 'color',[.8 .6 .6], 'markersize', 5, 'MarkerFaceColor',[.8 .6 .6])     
            
        %Plotting Foot Curves:
        %      rim_dat = [rim_dat; d_a, theta_a, d_a2, theta_a2];
        for z = 3:length(qf)           
            %Stance Foot Rim:           
            plot([(LR*sin(q(1,loop))) + rim_dat(z,1)*cos( rim_dat(z,2) - q(1,loop) -theta ), (LR*sin(q(1,loop))) + rim_dat(z-1,1)*cos( rim_dat(z-1,2) - q(1,loop) -theta )], ...
          [height-(LR*cos(q(1,loop)) + rim_dat(z,1)*sin( rim_dat(z,2) - q(1,loop) -theta )), height-(LR*cos(q(1,loop)) + rim_dat(z-1,1)*sin( rim_dat(z-1,2) - q(1,loop) -theta ))],...
                 '-k','Linewidth',3);;
            %Swing Foot Rim:
             plot([(LL*sin(q(2,loop))) + rim_dat(z,3)*cos( rim_dat(z,4) - q(2,loop) -theta  ), (LL*sin(q(2,loop))) + rim_dat(z-1,3)*cos( rim_dat(z-1,4) - q(2,loop) -theta )], ...
           [height-(LL*cos(q(2,loop)) + rim_dat(z,3)*sin( rim_dat(z,4) - q(2,loop) -theta  )), height-(LL*cos(q(2,loop)) + rim_dat(z-1,3)*sin( rim_dat(z-1,4) - q(2,loop) -theta ))],...
                 '-k','Linewidth',3); 
        end

        %----------------------

        %Hip Mass:
        plot(0,height,'.k','MarkerSize',max([mh*mhmul 1]),'EraseMode','none')
        %Link 1 masses: (Stance Foot)
        %ms1R at (a1R)
        plot((LR-a1R)*sin(q(1,loop)),height-(LR-a1R)*cos(q(1,loop)),'.k','MarkerSize',max([ms1R*msmul 1]))
        %ms2R at (a1R+b1R)
        plot((LR-a1R-b1R)*sin(q(1,loop)),height-(LR-a1R-b1R)*cos(q(1,loop)),'.k','MarkerSize',max([ms2R*msmul 1]))
        %mt2R at (lsR+c2R)
        plot((LR-lsR-c2R)*sin(q(1,loop)),height-(LR-lsR-c2R)*cos(q(1,loop)),'.k','MarkerSize',max([mt2R*mtmul 1]))
        %mt1R at (lsR+c2R+b2R)
        plot((LR-lsR-c2R-b2R)*sin(q(1,loop)),height-(LR-lsR-c2R-b2R)*cos(q(1,loop)),'.k','MarkerSize',max([mt1R*mtmul 1]))
        
        
        %Link 2 masses:  (Swing Thigh)
        %mt1L at (a2L)
        plot([(a2R)*sin(q(2,loop))],height+[-(a2R)*cos(q(2,loop))],'.k','MarkerSize',max([mt1L*mtmul 1]))
        %mt2L at (a2L+b2L)
        plot([(a2L+b2L)*sin(q(2,loop))],height+[-(a2L+b2L)*cos(q(2,loop))],'.k','MarkerSize',max([mt2L*mtmul 1]))
        %ms2L at (ltL+c1L)
        plot([(ltL+c1L)*sin(q(2,loop))],...
     height+[-(ltL+c1L)*cos(q(2,loop))],'.k','MarkerSize',max([ms2L*msmul 1]))
        %ms1L at (ltL+c1L+b1L)
        plot([(ltL+c1L+b1L)*sin(q(2,loop))],...
     height+[-(ltL+c1L+b1L)*cos(q(2,loop))],'.k','MarkerSize',max([ms1L*msmul 1]))

        %----------------------
        
        %Global Time Display
        if (save_walking_figure == 0)
            text(-0.9, 2.35, 'Elapsed Time:', 'FontSize',18,  'HorizontalAlignment', 'right');
            text(-0.85, 2.35, num2str(realtime(length(realtime)-length(time)) + realtime(loop)), 'FontSize',19, 'HorizontalAlignment','left');

            text(-0.9, 2.175, 'Stride:', 'FontSize',18,  'HorizontalAlignment', 'right');
            text(-0.85, 2.175, num2str(steps), 'FontSize',19, 'HorizontalAlignment','left');

            %Phase Text
            text(-0.9, 2.00, 'Phase:', 'FontSize',18,  'HorizontalAlignment', 'right');
            text(-0.85, 2.00, 'Right Stance', 'FontSize',18, ...
                'BackgroundColor',[.2 .6 .8], 'HorizontalAlignment','left',...
                'EdgeColor','k', 'LineWidth',1,  'Margin',2);
        end
        
        drawnow
        hold off
        
        
        %limit limit_cycle plot
        if (limit_cycle) && (loop > n+1)
            set(0, 'CurrentFigure', h2)
            hold on
            set(h2,'DoubleBuffer','on');
            plot([q(1,n), q(1,1)], [qd(1,n), qd(1,1)], '-b', 'LineWidth', 2)
            plot(q(1,:),qd(1,:),':b')
            plot([q(1,loop), q(1,loop-n)], [qd(1,loop), qd(1,loop-n)], '-b', 'LineWidth', 2)
            drawnow
        end
        
        
        %Saving animation as .jpg pictures
        if (save_walking_figure)  && (steps > 3) && (steps < 5)
            set(gca,'XTick',[])
            set(gca,'YTick',[])

            fprintf('\nSaving Animation Image Number: %i', print_count);
            
            set(gcf,'PaperPositionMode','auto')
            print('-djpeg','-r200',strcat(['C:\Users\ihandzic\Dropbox\REED LAB\Ismet\PDW\Ismet\PDW Simulation Model\PDW Curved Foot Model\PDW Curved Foot Three Link (9-mass)\Animation Video\LR2\' num2str(print_count) '.jpg']))

            pause(0.3)
            
            print_count = print_count + 1;
        end
        
    end
end




%=====================    COLLISION EVENTS    ========================
%                           HEEL STRIKE
%                           RIGHT STANCE
%=====================================================================
q1 = q(1,end);       q2 = q(2,end);
rL = rL(end);       rR = rR(end);

%Calling Heel Strike Function at the end of RIGHT stance
[Qm2, Qp2] =        Heel_RIGHT(q1, q1d, q2, q2d, FootData, rL, rR, parameters, psi);

qdm = [qd(1,end); qd(2,end)];
qdp = Qp2^(-1)*Qm2*qdm;

q = q(:,end);
