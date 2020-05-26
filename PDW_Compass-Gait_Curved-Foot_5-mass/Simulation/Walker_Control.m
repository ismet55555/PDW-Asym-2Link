function [p, results] = Walker_Control(p)
% Full PDW Dynamics Model
% This model describes a passive dynamic walker in terms of each individual legs

% Message logger object reference
global log

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%    Assigning all Incoming Parameters    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
g             = p.sim.g;
dt            = p.sim.dt;
theta         = p.sim.theta;
total_strides = p.sim.total_strides;

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

rLa = p.walker.left.rLa;
rLb = p.walker.left.rLb;
dL  = p.walker.left.dL;

rRa = p.walker.right.rRa;
rRb = p.walker.right.rRb;
dR  = p.walker.right.dR;


% Left Leg Length
p.walker.left.LL  = p.walker.left.a1L  + p.walker.left.b1L  + p.walker.left.c1L;
LL = p.walker.left.LL;

% Right Leg Length
p.walker.right.LR = p.walker.right.a1R + p.walker.right.b1R + p.walker.right.c1R;
LR = p.walker.right.LR;


% Creating the results variables
results.sim.time   = [];

results.motion.left               = struct;
results.motion.right              = struct;
results.motion.all.left.q.data    = [];
results.motion.all.left.qd.data   = [];
results.motion.all.left.qdd.data  = [];
results.motion.all.right.q.data   = [];
results.motion.all.right.qd.data  = [];
results.motion.all.right.qdd.data = [];

results.energy.left_right = struct;
results.energy.left_right = struct;
results.energy.all.total.data = [];
results.energy.all.KE.data    = [];
results.energy.all.PE.data    = [];

results.force.left        = struct;
results.force.right       = struct;
results.force.all.Rx.data = [];
results.force.all.Ry.data = [];

results.collision.heel.left                 = struct;
results.collision.heel.right                = struct;
results.collision.heel.all.left.time_abs    = [];
results.collision.heel.all.left.q           = [];
results.collision.heel.all.left.qd_pre      = [];
results.collision.heel.all.left.qd_post     = [];
results.collision.heel.all.right.time_abs   = [];
results.collision.heel.all.right.q          = [];
results.collision.heel.all.right.qd_pre     = [];
results.collision.heel.all.right.qd_post    = [];

results.other.left_right.hip_height = struct;
results.other.all.hip_height.data   = [];


% Failure diagnostics
results.fail.fail          = false;
results.fail.too_long      = false;
results.fail.fell_forward  = false;
results.fail.fell_backward = false;
results.fail.phase         = -1;


% Step length per walker ste
results.step_length.left  = [];       
results.step_length.right = [];


% Flag for when simulatin has stopped (Once it becomes true code will stop)
results.sim.stopped	= false;

% Flag signaling that the walker strides completed is the total number specified
results.sim.success = false; 

% Current walker stride
results.sim.stride  = 0;
results.sim.strides = [];

% Current walker step
results.sim.step  = 0;
results.sim.steps = [];
    
% Initial Leg Angular Position
q  = [p.walker.init.q1; 
      p.walker.init.q2];
  
% Initial Leg Angular Velocity
qd = [p.walker.init.qd1;
      p.walker.init.qd2];

  

 
% Loop the predetrimined number of strides
log.info('Starting walker step sequence ...')
for stride = 1 : p.sim.total_strides
    % Logging the current stride number
    results.sim.stride          = stride;
    results.sim.strides(stride) = stride;
    
    % Incrementing and loggin step number
    results.sim.step                    = results.sim.step + 1;
    results.sim.steps(results.sim.step) = results.sim.step;
    log.info(sprintf('Stride: %i - Step: %i - Left Stance', stride, results.sim.step))
    
    
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    Left Stance    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %Calls dynamics of two link LEFT phase (LEFT foot is on ground)
    [q, qdp, p, results] = Stance_LEFT(q, qd, p, results);
        
    %Check whether walker is still walking
    if(results.sim.stopped)
        break;
	end
       
    % Compute the effective foot shapes accross the stance phase
    rL = rLa + rLb*q(1, end);
    rR = rRa + rRb*q(2, end);
    
    % Geometry Calculations
    [psi, psi2, d_a, phi, theta_a, d_a2, phi2, theta_a2, height] ...
        = gContact('L', p, q(1,end), q(2,end), rL, rR, rLb, rRb, LL, LR);
    
	% Computing effective leg length
    results.step_length.left(stride) = abs( (LL*sin(q(1, end) + d_a *sin(pi - (pi/2 - q(1, end)) - theta_a ) ))...
                                          - (LR*sin(q(2, end) + d_a2*sin(pi - (pi/2 - q(2, end)) - theta_a2) )));

						
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    Right Stance    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Incrementing and loggin step number
    results.sim.step                    = results.sim.step + 1;
    results.sim.steps(results.sim.step) = results.sim.step;  
    log.info(sprintf('Stride: %i - Step: %i - Right Stance', stride, results.sim.step))
    
    % Switch Angular Position and Velocity References
    q  = [q(2, end); q(1, end)];
    qd = [qdp(1);   -qdp(2)];
    
    %Calls dynamics of two link RIGHT phase (RIGHT foot is on ground)
    [q2, qdp, p, results] = Stance_RIGHT(q, qd, p, results);

    %Check whether walker is still walking
    if(results.sim.stopped)
        break;
    end
    
    
    % Compute the effective foot shapes accross the stance phase 
    rR = rRa + rRb*q2(1, end);    
    rL = rLa + rLb*q2(2, end);

    % Geometry Calculations
    [psi, psi2, d_a, phi, theta_a, d_a2, phi2, theta_a2, height] ...
        = gContact('R', p, q2(1,end), q2(2,end), rR, rL, rRb, rLb, LR, LL);

	% Computing effective leg length
    results.step_length.right(stride) = abs( (LR*sin(q2(1, end) + d_a *sin(pi - (pi/2 - q2(1, end)) - theta_a ) ))...
                                           - (LL*sin(q2(2, end) + d_a2*sin(pi - (pi/2 - q2(2, end)) - theta_a2) )) );

    
	%-------------------------------------------------------------------------------------
	
	
    % Switch the Angular Positions (q) and Velocities (qd)
	% (Left becomes right or right becomes left)
    q  = [q2(2);   q2(1)];
    qd = [qdp(1); -qdp(2)];
    
    % Check if walker completed total stride initially specified
	if(stride >= p.sim.total_strides)
		results.sim.success = true;   
	end
end

% Save time length of simulation
results.sim.duration = results.sim.time(end);

end
