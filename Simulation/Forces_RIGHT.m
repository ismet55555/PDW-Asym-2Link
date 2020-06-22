function [Rx, Ry] = Forces_RIGHT(q, qd, qdd, p, r)
% This function computes the ground reaction forces for RIGHT stance for
% current walker origntation
% NOTE:
%       Rx - Forces along/parallel to ramp/ground
%       Ry - Forces perpendicular to ramp/ground

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%    Assigning all Incoming Parameters    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
g     = p.sim.g;
theta = p.sim.theta;

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


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%    Geometry Calculations    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[psi, psi2, d_a, phi, theta_a, d_a2, phi2, theta_a2, height] ...
    = gContact('R', p, q(1), q(2), r, 0, rRb, 0, LR, LL);

%ms1R at (a1R) from foot
dm_ms1R   = sqrt(a1R^2 + d_a^2 + 2*a1R*d_a*sin(theta_a));
beta_ms1R = - asin((a1R*sin(pi - theta_a + pi/2)) / dm_ms1R);

%mt1R at (a1R + b1R) from foot
dm_mt1R   = sqrt((a1R + b1R)^2 + d_a^2 + 2*(a1R + b1R)*d_a*sin(theta_a));
beta_mt1R = - asin(((a1R + b1R)*sin(pi - theta_a + pi/2)) / dm_mt1R);

%hip mass (mh) at (LR) from foot
dm_LR   = sqrt(LR^2 + d_a^2 + 2*LR*d_a*sin(theta_a));
beta_LR = -asin((LR*sin(pi - theta_a + pi/2))/dm_LR);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%  Ground Reaction Forces     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%        Statics              %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Rx = 0;
Ry = 0;

% ---------------------  Forces along/parallel to the ramp   -----------------------------
%  Left Leg (leg with ground contact) (distances from ground)
Rx = Rx + ms1R*g*sin(theta);  %mass ms1R at a1R from foot
Rx = Rx + mt1R*g*sin(theta);  %mass mt1R at a1R+b1R from foot

%  Hip
Rx = Rx + mh*g*sin(theta);  % mass mh at LR

% Right Leg (swing leg) (distances from hip)
Rx = Rx + mt1L*g*sin(theta);  % mass mt1L at c1L from hip
Rx = Rx + ms1L*g*sin(theta);  % mass ms1L at c1L+b1L from hip


% ---------------------  Forces perpendicular to the ramp   ------------------------------
%  Left Leg (leg with ground contact) (distances from ground)
Ry = Ry + ms1R*g*cos(theta);  % mass ms1R at a1R from foot
Ry = Ry + mt1R*g*cos(theta);  % mass mt1R at a1R+b1R from foot
%  Hip
Ry = Ry + mh*g*cos(theta);  % mass mh at LR
% Right Leg (swing leg) (distances from hip)
Ry = Ry + mt1L*g*cos(theta);  % mass mt1L at c1L from hip
Ry = Ry + ms1L*g*cos(theta);  % mass ms1L at c1L+b1L from hip


% Assigning to temporary variables
Rx2RS = Rx;
Ry2RS = Ry;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%  Ground Reaction Forces     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%        Dynamics             %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% NOTE:
%    Normal Acceleration     = a_n = L*qd^2
%    Tangential Acceleration = a_t = L*qdd
%    Total Relative Acceleration of swing mass = 
%                 a_swingmass - ground = a_hip-ground + a_swingmass - hipRx = 0;
Rx = 0;
Ry = 0;

% ---------------------  Forces along/parallel to the ramp   -----------------------------
%  RIGHT LEG (leg with ground contact) (distances from ground)
%mass ms1R at a1R from foot
a_n1 = dm_ms1R*qd(1)^2;  
a_t1 = dm_ms1R*qdd(1);   
Rx = Rx + ms1R*(- a_n1*sin(beta_ms1R - phi) - a_t1*cos(beta_ms1R - phi));

%mass mt1R at a1R+b1R from foot
a_n1 = dm_mt1R*qd(1)^2;  
a_t1 = dm_mt1R*qdd(1);
Rx = Rx + mt1R*(- a_n1*sin(beta_mt1R-phi) - a_t1*cos(beta_mt1R - phi));

%  HIP
%mass mh at LL from foot
a_n1 = dm_LR*qd(1)^2;  
a_t1 = dm_LR*qdd(1);
Rx = Rx + mh*(- a_n1*sin(beta_LR - phi) - a_t1*cos(beta_LR - phi));

% RIGHT LEG (swing leg) (distances from hip)
%mt1L at (c1L) from hip 
a_n2 = (c1L)*qd(2)^2;  
a_t2 = (c1L)*qdd(2);
Rx = Rx + mt1L*(- a_n1*sin(beta_LR - phi) - a_t1*cos(beta_LR - phi) ...
                + a_n2*sin(q(2) - theta ) + a_t2*cos(q(2) - theta));     

%ms1L at (c1L + b1L) from hip
a_n2 = (c1L + b1L)*qd(2)^2;  
a_t2 = (c1L + b1L)*qdd(2);
Rx = Rx + ms1L*(- a_n1*sin(beta_LR - phi) - a_t1*cos(beta_LR - phi) ...
                + a_n2*sin(q(2) - theta ) + a_t2*cos(q(2) - theta));                    



% ---------------------  Forces perpendicular to the ramp   ------------------------------
%  RIGHT LEG (leg with ground contact) (distances from ground)
%mass ms1R at a1R from foot
a_n1 = dm_ms1R*qd(1)^2;  
a_t1 = dm_ms1R*qdd(1); 
Ry = Ry + ms1R*(- a_n1*cos(beta_ms1R - phi) - a_t1*sin(beta_ms1R - phi));     

%mass mt1L at a1L+b1L from foot
a_n1 = dm_mt1R*qd(1)^2;  
a_t1 = dm_mt1R*qdd(1);
Ry = Ry + mt1R*(- a_n1*cos(beta_mt1R - phi) - a_t1*sin(beta_mt1R - phi));    

%  HIP
%mass mh at LR from foot
a_n1 = dm_LR*qd(1)^2;  
a_t1 = dm_LR*qdd(1);
Ry = Ry + mh*(- a_n1*cos(beta_LR - phi) - a_t1*sin(beta_LR - phi)); 

% RIGHT LEG (swing leg) (distances from hip)
%mt1L at (c1L) from hip 
a_n2 = (c1L)*qd(2)^2;  
a_t2 = (c1L)*qdd(2);
Ry = Ry + mt1L*(- a_n1*cos(beta_LR - phi) - a_t1*sin(beta_LR - phi) ...     
                + a_n2*cos(q(2) - theta ) + a_t2*sin(q(2) - theta ));

%mS1L at (c1L+b1L) from hip 
a_n2 = (c1L + b1L)*qd(2)^2;  
a_t2 = (c1L + b1L)*qdd(2);
Ry = Ry + ms1L*(- a_n1*cos(beta_LR - phi) - a_t1*sin(beta_LR - phi) ...        
                + a_n2*cos(q(2) - theta ) + a_t2*sin(q(2) - theta ));


% Assigning to temporary variables
Rx2RD = Rx;
Ry2RD = Ry;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%  Combined Ground Reaction Forces     %%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Reaction in X on left heel
Rx = (Rx2RS + Rx2RD);

%Reaction in Y on left heel
Ry = (Ry2RS + Ry2RD);

end

