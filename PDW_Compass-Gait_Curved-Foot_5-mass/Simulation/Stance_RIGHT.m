function [q, qdp, p, results] = Stance_RIGHT(q, qd , p, results)
% STANCE_RIGHT: Computes all derived parameters during 
%               RIGHT stance (right foot is on ground)


% Message logger object reference
global log

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%    Assigning all Incoming Parameters    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
dt          = p.sim.dt;
theta       = p.sim.theta;

mh   = p.walker.hip.mh;

mt1L = p.walker.left.mt1L;
ms1L = p.walker.left.ms1L;
a1L  = p.walker.left.a1L;
b1L  = p.walker.left.b1L;
c1L  = p.walker.left.c1L;

mt1R = p.walker.right.mt1R;
ms1R = p.walker.right.ms1R;
a1R  = p.walker.right.a1R;
b1R  = p.walker.right.b1R;
c1R  = p.walker.right.c1R;

LL  = p.walker.left.LL;
LR  = p.walker.right.LR;

rLa = p.walker.left.rLa;
rLb = p.walker.left.rLb;
dL  = p.walker.left.dL;

rRa = p.walker.right.rRa;
rRb = p.walker.right.rRb;
dR  = p.walker.right.dR;

mh_mul = p.walker.animation.mh_mul;
mt_mul = p.walker.animation.mt_mul;
ms_mul = p.walker.animation.ms_mul;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%    Creating Foot Shape Rim (Curve)    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
log.debug("Right Stance - Creating left foot shape ...")
% Define Left Foot radii (Stance Foot)
rL = zeros(1, 10000);
rL(1) = rLa + rLb*q(1, end);

% Define Right Foot radii (Swinging Foot)
rR = zeros(1, 10000);
rR(1) = rRa + rRb*q(2, end);

qf = [linspace(pi/2, -pi/2, 15); linspace(pi/2, -pi/2, 15)]; 
rLf = ones(1,length(qf))*rLa + rLb*qf(2,:);  % Swing (left) foot
rRf = ones(1,length(qf))*rRa + rRb*qf(1,:);  % Stance (right) foot

rim_dat = zeros(length(qf), 4);
for i = 1:length(qf)
   [psi, psi2, d_a, phi, theta_a, d_a2, phi2, theta_a2, hip_height] ...
       = gContact('R', p, qf(1,i), qf(2,i), rRf(i), rLf(i), 0, 0, LR, LL);
   
   rim_dat(i, :) = [d_a, theta_a, d_a2, theta_a2];
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%    Geometry Calculations    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
log.debug("Right Stance - Calculating initial walker geometery paramters ...")
[psi, psi2, d_a, phi, theta_a, d_a2, phi2, theta_a2, hip_height(1)] ...
    = gContact('R', p, q(1,end), q(2,end), rR(1), rL(1), rRb, rLb, LR, LL);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    Simulation Loop    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
results.sim.stopped = 0;

dt_dumdum	= dt; 
ramp		= -100;  % Initial ramp hip_height
time(1)		= 0;	 % Initial stance phase time
count		= 2;

% Keep looping as long as the lowest point of the foot is above the ramp
log.debug("Right Stance - Starting simulation loop ...")
while((hip_height(count - 1) - LL*cos(q(2, count - 1)) - d_a2*sin(theta_a2 - q(2, count - 1))) > ramp) 
	
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   Finding Ramp Position    %%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	% Calculating leg lengths
	lengthLL = LL*sin(q(2, count - 1)) + d_a2*cos(theta_a2 - q(2, count - 1));
    lengthLR = LR*sin(q(1, count - 1)) + d_a *cos(theta_a  - q(1, count - 1));
	
	% Ramp Position
    ramp = -(-lengthLR + lengthLL)*tan(theta);
	
	
	% If swinging foot is moving forward or if simulation is in initial
	% phase (count less than 10) ensure walker does not skuff foot on ground
    if qd(2,count-1) > 0  
        ramp = ramp - 0.4;		% Decrease the ramp angle threshold
        dt = dt_dumdum + 0.003;	% Increase the Simulation resolution
    else
        dt = dt_dumdum;			% Reset (Reduce) the simulation resolution
	end
	
	% Conenience variable redefinition of angular position and velocities
	% Left Leg (Stance Leg)
    q1  = q(1, count - 1);
    q1d = qd(1, count - 1);
	% Right Leg (Swinging Leg)
    q2  = q(2, count - 1);
    q2d = qd(2, count - 1);

	
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   Kinematics Calculations    %%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Angular Accelerations (Lagrange)
    [qdd(:, count)] = Dynamics_2L_RIGHT(q1, q1d, q2, q2d, p, psi);
    
    % Angular Velocities (Numerical Integration)
    qd(:,count) = qd(:, count - 1) + qdd(:, count)*dt;

    % Angular Positions (Numerical Integration)
    q(:, count) = q(:,count - 1) + qd(:, count)*dt;
    

    %Increase simulation time step
    time(count) = time(count - 1) + dt;  
    
    %Define Foot Radius for current step
    rR(count) = rRa + rRb*q(1, count);  % Rig./ :L]ht foot radius
    rL(count) = rLa + rLb*q(2, count);  % Left foot radius

    
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%   Energy Calculations    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % TODO: Allocate and remove -1 at the end
    [E_total(count), KE(count), PE(count)] = Energy_RIGHT(q(:,end), qd(:,end), p, rR(count));
    
    
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%   Kinetics/Forces Calculations    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
     % TODO: Allocate and remove -1 at the end
    [Rx(count), Ry(count)] = Forces_RIGHT(q(:,end), qd(:,end), qdd(:,end), p, rR(count));

    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%%%%%%   Geometry Calculations    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    [psi, psi2, d_a, phi, theta_a, d_a2, phi2, theta_a2, hip_height(count)] ...
        = gContact('R', p, q(1,count), q(2,count), rR(count), rL(count), rRb, rLb, LR, LL);

    
    %Increase simulation step counter
    count = count + 1;
    
    
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%   Walker Failure Diagnostics    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %Check if the i exceeds iteration threshhold for gait phase 
    if count > 25000
        log.warning("Right Stance - Walker simulation exceeded 25000 iterations")
        results.fail.too_long  = true;
        results.sim.stopped    = true;
        break;
	end
	
	% Check if walker fell forward
    if q(1, count - 1) < -pi/2 + pi/6
        log.warning("Right Stance - Walker fell forward")
        results.fail.fell_forward = true;
        results.sim.stopped       = true;
        break;
	end
	
	% Check if walker fell backward
    if q(1, count - 1) > (pi - theta) - pi/6
        log.warning("Right Stance - Walker fell backward")
        results.fail.fell_backward = true;
        results.sim.stopped        = true;
        break;
    end
end
log.debug("Right Stance - Simulation loop ended")

% Check if model ran few simuilation results.sim.step 
if(length(q) < 20)
    log.warning(sprintf("Right Stance - Walker simulation ended after only %i iterations", length(q)))
    results.fail.tripped = true;
    results.sim.stopped  = 1;
end

% Remove first entries, they are included within the last phase
time(1)       = [];
hip_height(1) = [];
q(:, 1)       = [];
qd(:, 1)      = [];
qdd(:, 1)     = [];
E_total(1)    = [];
PE(1)         = [];
KE(1)         = [];
Rx(1)         = [];
Ry(1)         = [];

%Deleting zeros in radius vector
rL(rL == 0) = [];
rR(rR == 0) = [];


log.debug("Right Stance - Organizing and saving derived parameters ...")

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%    Save Leg Angular Dynamics    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ----------------------------  Store Cummalative Data  ----------------------------------
% RIGHT leg is STANCE leg (q1, qd1, qdd1)
results.motion.all.right.q.data   = [results.motion.all.right.q.data,   q(1, :)  ];
results.motion.all.right.qd.data  = [results.motion.all.right.qd.data,  qd(1, :) ];
results.motion.all.right.qdd.data = [results.motion.all.right.qdd.data, qdd(1, :)];
% LEFT leg is SWING leg (q2, qd2, qdd2)
results.motion.all.left.q.data   = [results.motion.all.left.q.data,   q(2, :)  ];
results.motion.all.left.qd.data  = [results.motion.all.left.qd.data,  qd(2, :) ];
results.motion.all.left.qdd.data = [results.motion.all.left.qdd.data, qdd(2, :)];

% ----------------------------  Store Per-Step Data  -------------------------------------
% RIGHT leg is STANCE leg (q1, qd1, qdd1)
[results.motion.right] = store_per_step_data(results.motion.right, "q",   q(1, :),   time, results.sim.step);
[results.motion.right] = store_per_step_data(results.motion.right, "qd",  qd(1, :),  time, results.sim.step);
[results.motion.right] = store_per_step_data(results.motion.right, "qdd", qdd(1, :), time, results.sim.step);
% LEFT leg is SWING leg (q2, qd2, qdd2)
[results.motion.left] = store_per_step_data(results.motion.left, "q",   q(2, :),   time, results.sim.step);
[results.motion.left] = store_per_step_data(results.motion.left, "qd",  qd(2, :),  time, results.sim.step);
[results.motion.left] = store_per_step_data(results.motion.left, "qdd", qdd(2, :), time, results.sim.step);


% Save the motion phases for this step
results.motion.right.phase{results.sim.step} = "stance";  % Stance phase
results.motion.left.phase{results.sim.step}  = "swing";   % Swing phase


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    Walker Energy Data    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ----------------------------  Store Cummalative Data  ----------------------------------
results.energy.all.total.data = [results.energy.all.total.data, E_total];
results.energy.all.KE.data    = [results.energy.all.KE.data,    KE];
results.energy.all.PE.data    = [results.energy.all.PE.data,    PE];

% ----------------------------  Store Per-Step Data  -------------------------------------
[results.energy.left_right] = store_per_step_data(results.energy.left_right, "total", E_total, time, results.sim.step);
[results.energy.left_right] = store_per_step_data(results.energy.left_right, "KE",    KE,      time, results.sim.step);
[results.energy.left_right] = store_per_step_data(results.energy.left_right, "PE",    PE,      time, results.sim.step);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%    Walker Kinetic/Forces Data    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ----------------------------  Store Cummalative Data  ----------------------------------
results.force.all.Rx.data    = [results.force.all.Rx.data, Rx];
results.force.all.Ry.data    = [results.force.all.Ry.data, Ry];

% ----------------------------  Store Per-Step Data  -------------------------------------
[results.force.right] = store_per_step_data(results.force.right, "Rx", Rx, time, results.sim.step);
[results.force.right] = store_per_step_data(results.force.right, "Ry", Ry, time, results.sim.step);
[results.force.left] = store_per_step_data(results.force.left, "Rx", zeros(1, length(Rx)), time, results.sim.step);
[results.force.left] = store_per_step_data(results.force.left, "Ry", zeros(1, length(Rx)), time, results.sim.step);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    Save Other Parameters    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ----------------------------  Store Cummalative Data  ----------------------------------
results.other.all.hip_height.data = [results.other.all.hip_height.data, hip_height];

% ----------------------------  Store Per-Step Data  -------------------------------------
[results.other.left_right] = store_per_step_data(results.other.left_right, "hip_height", hip_height, time, results.sim.step);



% Add to global simuilation time 
if isempty(results.sim.time)
    results.sim.time = [results.sim.time, time];
else
    results.sim.time = [results.sim.time, results.sim.time(end) + time];
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%    Simulation Animation Display    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if(p.sim.output.animation)
    log.debug("Right Stance - Showing walker simulation animation ...")
    try
       % Probe to see if animtion figure handle exists
       p.sim.figure_handles.animation;
    catch
       % If it didn't exist create a new figure for it
       p.sim.figure_handles.animation = figure('Name' ,"PDW: Animation", ...
                                               'NumberTitle','off');

       % Format the figure for the first time
        set(p.sim.figure_handles.animation,'color', 'w');
        set(p.sim.figure_handles.animation,'DoubleBuffer','on');
        set(p.sim.figure_handles.animation, 'position',[0, 0, 863, 677])
        set(p.sim.figure_handles.animation, 'MenuBar', 'none');
        set(p.sim.figure_handles.animation, 'ToolBar', 'none');
        plot(NaN, NaN)
        hold on
        title("Animation View")
        grid on
        box on
        xlim([-1.75, 1.75])
        ylim([-0.50, 2.50])
		daspect([1, 1, 1])
    end
    animation_figure = p.sim.figure_handles.animation;
    set(0, 'CurrentFigure', animation_figure)
    hold on
        
	%Animation step invterval
    n = 10;
	
    for i = [1 : n : length(q), length(q)]
        % Clear the figure axis, without clearing formatting
        cla(animation_figure.CurrentAxes);
        
	   % Geometry calculations
        [psi, psi2, d_a, phi, theta_a, d_a2, phi2, theta_a2, hip_height] ...
            = gContact('R', p, q(1,i), q(2,i), rR(i), rL(i), rRb, rLb, LR, LL);

		%---------------------------------------------------------------------------------
        %-------------------------------------  Ramp  ------------------------------------
		%---------------------------------------------------------------------------------
        plot([LR*sin(q(1,i))- 3*cos(theta) + d_a*sin(pi-(pi/2 - q(1,i))-theta_a), LR*sin(q(1,i))+3*cos(theta)+ d_a*sin(pi-(pi/2 - q(1,i))-theta_a)], ...
            hip_height+[-LR*cos(q(1,i)) + 3*sin(theta) - d_a*cos(pi-(pi/2 - q(1,i))-theta_a), -LR*cos(q(1,i))-3*sin(theta)-d_a*cos(pi-(pi/2 - q(1,i))-theta_a) ],...
            '-k','LineWidth',3)
        patch([LR*sin(q(1,i))- 3*cos(theta) + d_a*sin(pi-(pi/2 - q(1,i))-theta_a), LR*sin(q(1,i))+3*cos(theta)+ d_a*sin(pi-(pi/2 - q(1,i))-theta_a), 10, -10],...
          hip_height+[-LR*cos(q(1,i)) + 3*sin(theta) - d_a*cos(pi-(pi/2 - q(1,i))-theta_a), -LR*cos(q(1,i))-3*sin(theta)-d_a*cos(pi-(pi/2 - q(1,i))-theta_a), -5,-5 ],...
          [0.5,0.7,0.5] ) ;
        
      
		%---------------------------------------------------------------------------------
        %-------------------------------  Stance Leg  ------------------------------------
		%---------------------------------------------------------------------------------
        plot([0, LR*sin(q(1,i))], hip_height+[0, -LR*cos(q(1,i))],'color',[.2 .6 .8],'LineWidth',6)
        %Plot link 1 base to ground contact:
        plot([(LR*sin(q(1,i)))+ d_a*sin(pi-(pi/2 - q(1,i))-theta_a), LL*sin(q(1,i))], ...
             [hip_height-(LR*cos(q(1,i))+d_a*cos(pi-(pi/2 - q(1,i))-theta_a)), (hip_height-LL*cos(q(1,i)))] ,':b','LineWidth',2,'color',[.2 .6 .8])
        %Plot curve top for stance  
        plot([LR*sin(q(1,i))-(rRf(1)-dR)*sin(pi/2-q(1,i)) , LR*sin(q(1,i))+(dR+rRf(end))*sin(pi/2-q(1,i))],...
            [hip_height-LR*cos(q(1,i))-(rRf(1)-dR)*cos(pi/2-q(1,i)) , hip_height-LR*cos(q(1,i))+(dR+rRf(end))*cos(pi/2-q(1,i))],...
            '-k','LineWidth',4) 
        %right foot contact
        plot((LR*sin(q(1,i)))+ d_a*sin(pi-(pi/2 - q(1,i))-theta_a), ...
             hip_height-(LR*cos(q(1,i))+d_a*cos(pi-(pi/2 - q(1,i))-theta_a)), 'color',[.2 .6 .8], 'markersize', 5, 'MarkerFaceColor', [.2 .6 .8])

         
		%---------------------------------------------------------------------------------
        %-------------------------------  Swing Leg  ------------------------------------
		%---------------------------------------------------------------------------------
        plot([0, LL*sin(q(2,i))],hip_height+[0, -LL*cos(q(2,i))],'color',[.8 .6 .6],'LineWidth',6)
        %Plot link 2 base to shoe base
        plot([LL*sin(q(2,i)), (LL*sin(q(2,i)))+ d_a2*sin(pi-(pi/2 - q(2,i))-theta_a2)],...
                hip_height+[(-LL*cos(q(2,i))), -(LL*cos(q(2,i))+d_a2*cos(pi-(pi/2 - q(2,i))-theta_a2))],':r','LineWidth',2,'color',[.8 .6 .6])
        %Plot curve top for swing
        plot([LL*sin(q(2,i))-(rLf(1)-dL)*sin(pi/2-q(2,i)) , LL*sin(q(2,i))+(dL+rLf(end))*sin(pi/2-q(2,i))],...
            [hip_height-LL*cos(q(2,i))-(rLf(1)-dL)*cos(pi/2-q(2,i)) , hip_height-LL*cos(q(2,i))+(dL+rLf(end))*cos(pi/2-q(2,i))],...
            '-k','LineWidth',4)
        %left foot contact               
        plot((LL*sin(q(2,i)))+ d_a2*sin(pi-(pi/2 - q(2,i))-theta_a2), ...
             hip_height-(LL*cos(q(2,i))+d_a2*cos(pi-(pi/2 - q(2,i))-theta_a2)), 'color',[.8 .6 .6], 'markersize', 5, 'MarkerFaceColor',[.8 .6 .6])     
            
         
		%---------------------------------------------------------------------------------
        %-------------------------------------  Feet  ------------------------------------
		%---------------------------------------------------------------------------------
        %Plotting Foot Curves:
        %      rim_dat = [rim_dat; d_a, theta_a, d_a2, theta_a2];
        for z = 3:length(qf)           
            %Stance Foot Rim:           
            plot([(LR*sin(q(1,i))) + rim_dat(z,1)*cos( rim_dat(z,2) - q(1,i) -theta ), (LR*sin(q(1,i))) + rim_dat(z-1,1)*cos( rim_dat(z-1,2) - q(1,i) -theta )], ...
                 [hip_height-(LR*cos(q(1,i)) + rim_dat(z,1)*sin( rim_dat(z,2) - q(1,i) -theta )), hip_height-(LR*cos(q(1,i)) + rim_dat(z-1,1)*sin( rim_dat(z-1,2) - q(1,i) -theta ))],...
                 '-k','Linewidth',2)  
            %Swing Foot Rim:
             plot([(LL*sin(q(2,i))) + rim_dat(z,3)*cos( rim_dat(z,4) - q(2,i) -theta  ), (LL*sin(q(2,i))) + rim_dat(z-1,3)*cos( rim_dat(z-1,4) - q(2,i) -theta )], ...
                 [hip_height-(LL*cos(q(2,i)) + rim_dat(z,3)*sin( rim_dat(z,4) - q(2,i) -theta  )), hip_height-(LL*cos(q(2,i)) + rim_dat(z-1,3)*sin( rim_dat(z-1,4) - q(2,i) -theta ))],...
                 '-k','Linewidth',2)  
             
             drawnow limitrate
        end

        %---------------------------------------------------------------------------------
        %-----------------------------  Point Masses  ------------------------------------
		%---------------------------------------------------------------------------------
        %Hip Mass:
        plot(0,hip_height,'.k','MarkerSize',max([mh*mh_mul 1]))
        %Link 1 masses:
        plot((LR-a1R)*sin(q(1,i)),hip_height-(LR-a1R)*cos(q(1,i)),'.k','MarkerSize',max([ms1R*ms_mul 1]))
        plot((LR-a1R-b1R)*sin(q(1,i)),hip_height-(LR-a1R-b1R)*cos(q(1,i)),'.k','MarkerSize',max([mt1R*mt_mul 1]))
        %Link 2 masses:
        plot([(LL-a1L-b1L)*sin(q(2,i))],hip_height+[-(LL-a1L-b1L)*cos(q(2,i))],'.k','MarkerSize',max([mt1L*ms_mul 1]))
        plot([(LL-a1L)*sin(q(2,i))],hip_height+[-(LL-a1L)*cos(q(2,i))],'.k','MarkerSize',max([ms1L*ms_mul 1]))

        
        %---------------------------------------------------------------------------------
        %-----------------------------  Annotations  -------------------------------------
		%---------------------------------------------------------------------------------
        %Global Time
        text(-0.9, 2.35, 'Elapsed Time:', 'FontSize',18,  'HorizontalAlignment', 'right')
        text(-0.85, 2.35, num2str(results.sim.time(length(results.sim.time)-length(time)) + results.sim.time(i)), ...
             'FontSize',19, 'HorizontalAlignment','left')

		% Step Number
        text(-0.9, 2.175, 'Step:', 'FontSize',18,  'HorizontalAlignment', 'right')
        text(-0.85, 2.175, num2str(results.sim.step), 'FontSize',19, 'HorizontalAlignment','left')
        
        %Phase Text
        text(-0.9, 2.00, 'Phase:', 'FontSize',18,  'HorizontalAlignment', 'right')
        text(-0.85, 2.00, 'Right Stance', 'FontSize',18, ...
            'BackgroundColor',[.2 .6 .8], 'HorizontalAlignment','left',...
            'EdgeColor','k', 'LineWidth',1,  'Margin',2)
        
        
        % Update the walker figure
        drawnow

		
		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		%%%%%%%%%%%%%    Saving Animation Display Frame Image   %%%%%%%%%%%%%%%%%%%%%%%%%%
		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		%Saving animation as .jpg pictures to make create video after
		if (p.sim.output.save_walking_frames)
            %TODO: Check if directory exists, if not, create it
			print(h, "-djpeg", "-r300", join([p.sim.output.save_walking_frames_dir, ...
                                             num2str(p.sim.output.print_frame_index)], ""))    

            % Increment the global frame index that was saved
			p.sim.output.print_frame_index = p.sim.output.print_frame_index + 1;
        end
    end
else
    log.debug("Right Stance - Walker simulation animation skipped")
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%    Collision Event Calculations    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%      Right Stance, Heel Strike     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
log.debug("Right Stance - Calculating collision event (heel strike) ...")

q1  = q(1, end);
q1d = qd(1, end);
q2  = q(2, end);
q2d = qd(2, end);

% Last configuration of the foot radius
rL = rL(end);
rR = rR(end);

% Calling Heel Strike Function at the end of RIGHT stance
[Qm2, Qp2] = Collision_Heel_RIGHT(q1, q1d, q2, q2d, p, rL, rR, psi);

% Leg angular velocities before ("m") collision event
qdm = [qd(1, end); qd(2, end)];

% Leg angular velocities after ("p") collision event
qdp = Qp2^(-1)*Qm2*qdm;

% Leg angular positions after collision event
q = q(:, end);



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%    Save Collision Parameters    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
log.debug("Right Stance - Organizing and saving collision event parameters ...")

% ----------------------------  Store Cummalative Data  ----------------------------------
results.collision.heel.all.left.time_abs = [results.collision.heel.all.left.time_abs, results.sim.time(end)];
results.collision.heel.all.left.q        = [results.collision.heel.all.left.q,        q(2, end)];
results.collision.heel.all.left.qd_pre   = [results.collision.heel.all.left.qd_pre,   qdm(2)];
results.collision.heel.all.left.qd_post  = [results.collision.heel.all.left.qd_post,  qdp(2)];


% ----------------------------  Store Per-Step Data  -------------------------------------
results.collision.heel.right.time_rel{results.sim.step} = time(end);
results.collision.heel.right.time_abs{results.sim.step} = results.sim.time(end);
results.collision.heel.right.q{results.sim.step}        = q(1, end);
results.collision.heel.right.qd_pre{results.sim.step}   = qdm(1);
results.collision.heel.right.qd_post{results.sim.step}  = qdp(1);

results.collision.heel.left.time_rel{results.sim.step} = time(end);
results.collision.heel.left.time_abs{results.sim.step} = results.sim.time(end);
results.collision.heel.left.q{results.sim.step}        = q(2, end);
results.collision.heel.left.qd_pre{results.sim.step}   = qdm(2);
results.collision.heel.left.qd_post{results.sim.step}  = qdp(2);