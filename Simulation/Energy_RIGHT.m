function [E, KE, PE] = energy_RIGHT(q, qd, p, rR)
% RIGHT Stance walker energy calculations for current walker leg
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
    = gContact('R', p, q1, q2, rR, 0, 0, 0, LR, LL);

% ms1R at (a1R)
dm_a1R   = sqrt(a1R^2 + d_a^2 + 2*a1R*d_a*sin(theta_a));
beta_a1R = -asin((a1R*sin(pi - theta_a + pi/2))/dm_a1R);
psi_a1R  = -(beta_a1R-phi+theta);

% mt1R at (a1R + b1R)
dm_a1Rb1R   = sqrt((a1R + b1R)^2 + d_a^2 + 2*(a1R + b1R)*d_a*sin(theta_a));
beta_a1Rb1R = -asin(((a1R + b1R)*sin(pi - theta_a + pi/2))/dm_a1Rb1R);
psi_a1Rb1R  = -(beta_a1Rb1R-phi+theta);

% mh
dm_LR   = sqrt(LR^2 + d_a^2 + 2*LR*d_a*sin(theta_a));
beta_LR = -asin((LR*sin(pi - theta_a + pi/2))/dm_LR);
psi_LR  = -(beta_LR-phi+theta);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  Leg Velocities   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Right (stance) Leg
v_mh   = [-dm_LR*q1d*cos(psi_LR),          -dm_LR*q1d*sin(psi_LR)        ];  % mh - Hip mass
v_ms1R = [-dm_a1R*q1d*cos(psi_a1R),        -dm_a1R*q1d*sin(psi_a1R)      ];  % mt1R (upper thigh)
v_mt1R = [-dm_a1Rb1R*q1d*cos(psi_a1Rb1R),  -dm_a1Rb1R*q1d*sin(psi_a1Rb1R)];  % ms1R (lower shank)

%Left (swing) Leg
v_mt1L = [(c1L)*q2d*cos(q2) - dm_LR*q1d*cos(psi_LR),...
    (c1L)*q2d*sin(q2) - dm_LR*q1d*sin(psi_LR)    ];  % mt1L (upper thigh)
v_ms1L = [(c1L+b1L)*q2d*cos(q2) - dm_LR*q1d*cos(psi_LR),...
    (c1L+b1L)*q2d*sin(q2) - dm_LR*q1d*sin(psi_LR)];  % ms1L (lower shank)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   Kinetic Energy   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Velocity Magnitudes
% Stance leg masses
v_mh_mag   = sqrt( v_mh(1)^2   + v_mh(2)^2 );    % mh - Hip mass
v_ms1R_mag = sqrt( v_ms1R(1)^2 + v_ms1R(2)^2 );  % mt1R (upper thigh)
v_mt1R_mag = sqrt( v_mt1R(1)^2 + v_mt1R(2)^2 );  % ms1R (lower shank)

% Swing Leg Masses
v_mt1L_mag = sqrt( v_mt1L(1)^2 + v_mt1L(2)^2 );  % mt1L (upper thigh)
v_ms1L_mag = sqrt( v_ms1L(1)^2 + v_ms1L(2)^2 );  % ms1L (lower thigh)

KE =  0.5*mh*  (v_mh_mag^2) ...
    + 0.5*ms1R*(v_ms1R_mag^2) ...
    + 0.5*mt1R*(v_mt1R_mag^2) ...
    + 0.5*mt1L*(v_mt1L_mag^2) ...
    + 0.5*ms1L*(v_ms1L_mag^2);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   Potential Energy   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
PE =    mh*g* (dm_LR*cos(psi_LR)) ...  
    + ms1R*g* (dm_a1R*cos(psi_a1R)) ...             
    + mt1R*g* (dm_a1Rb1R*cos(psi_a1Rb1R)) ... 
    + mt1L*g* (dm_LR*cos(psi_LR) - (c1L)*cos(q2)) ...
    + ms1L*g* (dm_LR*cos(psi_LR) - (c1L+b1L)*cos(q2));


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   Total Energy   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
E = KE + PE;

end