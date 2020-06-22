function [E, KE, PE] = energy_LEFT(q, qd, p, rL)
% LEFT Stance walker energy calculations for current walker leg
% angular positions and angular velocities
% Caluclates Total, Kinetic, and Potential Energy for current time step.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%    Assigning all Incoming Parameters    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
g           = p.sim.g;
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


% Leg Angular Positions
q1 = q(1);
q2 = q(2);

% Leg Angular Velocities
q1d = qd(1);
q2d = qd(2);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%    Geometry Calculations    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[psi, psi2, d_a, phi, theta_a, d_a2, phi2, theta_a2, height] ...
    = gContact('L', p, q1, q2, rL, 0, 0, 0, LL, LR);

% ms1L at (a1L)
dm_a1L   = sqrt(a1L^2 + d_a^2 + 2*a1L*d_a*sin(theta_a));
beta_a1L = -asin((a1L*sin(pi - theta_a + pi/2))/dm_a1L);
psi_a1L  = -(beta_a1L-phi+theta);

% mt1L at (a1L + b1L)
dm_a1Lb1L   = sqrt((a1L + b1L)^2 + d_a^2 + 2*(a1L + b1L)*d_a*sin(theta_a));
beta_a1Lb1L = -asin(((a1L + b1L)*sin(pi - theta_a + pi/2))/dm_a1Lb1L);
psi_a1Lb1L  = -(beta_a1Lb1L-phi+theta);

% mh
dm_LL   = sqrt(LL^2 + d_a^2 + 2*LL*d_a*sin(theta_a));
beta_LL = -asin((LL*sin(pi - theta_a + pi/2))/dm_LL);
psi_LL  = -(beta_LL-phi+theta);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  Leg Velocities   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Left (stance) Leg
v_mh   = [-dm_LL*q1d*cos(psi_LL),          -dm_LL*q1d*sin(psi_LL)        ];  % mh - Hip mass
v_ms1L = [-dm_a1L*q1d*cos(psi_a1L),        -dm_a1L*q1d*sin(psi_a1L)      ];  % mt1L (upper thigh)
v_mt1L = [-dm_a1Lb1L*q1d*cos(psi_a1Lb1L),  -dm_a1Lb1L*q1d*sin(psi_a1Lb1L)];  % ms1L (lower shank)

%Right (swing) Leg
v_mt1R = [(c1R)*q2d*cos(q2) - dm_LL*q1d*cos(psi_LL),...
          (c1R)*q2d*sin(q2) - dm_LL*q1d*sin(psi_LL)    ];  %mt1R (upper thigh)
v_ms1R = [(c1R+b1R)*q2d*cos(q2) - dm_LL*q1d*cos(psi_LL),...
          (c1R+b1R)*q2d*sin(q2) - dm_LL*q1d*sin(psi_LL)];  %ms1R (lower shank)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   Kinetic Energy   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Velocity Magnitudes
% Stance leg masses
v_mh_mag   = sqrt( v_mh(1)^2   + v_mh(2)^2 );    % mh - Hip mass
v_ms1L_mag = sqrt( v_ms1L(1)^2 + v_ms1L(2)^2 );  % mt1L (upper thigh)
v_mt1L_mag = sqrt( v_mt1L(1)^2 + v_mt1L(2)^2 );  % ms1L (lower shank)

% Swing Leg Masses
v_mt1R_mag = sqrt( v_mt1R(1)^2 + v_mt1R(2)^2 );  % mt1R (upper thigh)
v_ms1R_mag = sqrt( v_ms1R(1)^2 + v_ms1R(2)^2 );  % ms1R (lower thigh)

KE =   0.5*mh  *(v_mh_mag^2  ) ...	
     + 0.5*ms1L*(v_ms1L_mag^2) ...
     + 0.5*mt1L*(v_mt1L_mag^2) ...
     + 0.5*mt1R*(v_mt1R_mag^2) ...
     + 0.5*ms1R*(v_ms1R_mag^2);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   Potential Energy   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
PE =    mh*g* (dm_LL    *cos(psi_LL)) ... 
    + ms1L*g* (dm_a1L   *cos(psi_a1L)) ...
    + mt1L*g* (dm_a1Lb1L*cos(psi_a1Lb1L)) ... 
    + mt1R*g* (dm_LL    *cos(psi_LL) - (c1R)*cos(q2)) ... 
    + ms1R*g* (dm_LL    *cos(psi_LL) - (c1R+b1R)*cos(q2));


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   Total Energy   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
E = KE + PE;

end