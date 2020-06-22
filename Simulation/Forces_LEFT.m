function [Rx, Ry] = Forces_LEFT(q, qd, qdd, p, r)
% This function computes the ground reaction forces for LEFT stance for
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
    = gContact('L', p, q(1), q(2), r, 0, rLb, 0, LL, LR);

%ms1L at (a1L) from foot
dm_ms1L   = sqrt(a1L^2 + d_a^2 + 2*a1L*d_a*sin(theta_a));
beta_ms1L =-asin((a1L*sin(pi - theta_a + pi/2)) / dm_ms1L);

%mt1L at (a1L + b1L) from foot
dm_mt1L   = sqrt((a1L + b1L)^2 + d_a^2 + 2*(a1L + b1L)*d_a*sin(theta_a));
beta_mt1L = -asin(((a1L + b1L)*sin(pi - theta_a + pi/2)) / dm_mt1L);

%hip mass (mh) at (LL) from foot
dm_LL   = sqrt(LL^2 + d_a^2 + 2*LL*d_a*sin(theta_a));
beta_LL = -asin((LL*sin(pi - theta_a + pi/2))/dm_LL);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%  Ground Reaction Forces     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%        Statics              %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Rx = 0;
Ry = 0;

% ---------------------  Forces along/parallel to the ramp   -----------------------------
%  Left Leg (leg with ground contact) (distances from ground)
Rx = Rx + ms1L*g*sin(theta);  % mass ms1L at a1L from foot
Rx = Rx + mt1L*g*sin(theta);  % mass mt1L at a1L+b1L from foot

%  Hip
Rx = Rx + mh*g*sin(theta);  % mass mh at LL

% Right Leg (swing leg) (distances from hip)
Rx = Rx + mt1R*g*sin(theta);  % mass mt1R at c1R from hip
Rx = Rx + ms1R*g*sin(theta);  % mass ms1R at c1R+b1R from hip


% ---------------------  Forces perpendicular to the ramp   ------------------------------
%  Left Leg (leg with ground contact) (distances from ground)
Ry = Ry + ms1L*g*cos(theta);  % mass ms1L at a1L from foot
Ry = Ry + mt1L*g*cos(theta);  % mass mt1L at a1L+b1L from foot

%  Hip
Ry = Ry + mh*g*cos(theta);  % mass mh at LL

% Right Leg (swing leg) (distances from hip)
Ry = Ry + mt1R*g*cos(theta);  % mass mt1R at c1R from hip
Ry = Ry + ms1R*g*cos(theta);  % mass ms1R at c1R+b1R from hip


% Assigning to temporary variables
Rx2LS = Rx;
Ry2LS = Ry;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%  Ground Reaction Forces     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%        Dynamics             %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% NOTE:
%    Normal Acceleration     = a_n = L*qd^2
%    Tangential Acceleration = a_t = L*qdd
%    Total Relative Acceleration of swing mass = 
%                 a_swingmass - ground = a_hip-ground + a_swingmass - hip
Rx = 0;
Ry = 0;

% ---------------------  Forces along/parallel to the ramp   -----------------------------
%  LEFT LEG (leg with ground contact) (distances from ground)
% mass ms1L at a1L from foot
a_n1 = dm_ms1L*qd(1)^2;  
a_t1 = dm_ms1L*qdd(1);   
Rx = Rx + ms1L*(- a_n1*sin(beta_ms1L - phi) - a_t1*cos(beta_ms1L - phi));

%mass mt1L at a1L+b1L from foot
a_n1 = dm_mt1L*qd(1)^2;  
a_t1 = dm_mt1L*qdd(1);
Rx = Rx + mt1L*(- a_n1*sin(beta_mt1L - phi) - a_t1*cos(beta_mt1L - phi));

%  HIP
%mass mh at LL from foot
a_n1 = dm_LL*qd(1)^2;  
a_t1 = dm_LL*qdd(1);
Rx = Rx + mh*(- a_n1*sin(beta_LL - phi) - a_t1*cos(beta_LL - phi));

% RIGHT LEG (swing leg) (distances from hip)
%mt1R at (c1R) from hip 
a_n2 = (c1R)*qd(2)^2;  
a_t2 = (c1R)*qdd(2);
Rx = Rx + mt1R*(- a_n1*sin(beta_LL - phi) - a_t1*cos(beta_LL - phi) ...
                + a_n2*sin(q(2) - theta ) + a_t2*cos(q(2) - theta));       

%ms1R at (c1R + b1R) from hip
a_n2 = (c1R + b1R)*qd(2)^2;  
a_t2 = (c1R + b1R)*qdd(2);
Rx = Rx + ms1R*(- a_n1*sin(beta_LL - phi) - a_t1*cos(beta_LL - phi) ...
                + a_n2*sin(q(2) - theta ) + a_t2*cos(q(2) - theta));                   



% ---------------------  Forces perpendicular to the ramp   ------------------------------
%  LEFT LEG (leg with ground contact) (distances from ground)
%mass ms1L at a1L from foot
a_n1 = dm_ms1L*qd(1)^2;  
a_t1 = dm_ms1L*qdd(1); 
Ry = Ry + ms1L*(- a_n1*cos(beta_ms1L - phi) - a_t1*sin(beta_ms1L - phi));     

%mass mt1L at a1L+b1L from foot
a_n1 = dm_mt1L*qd(1)^2;  
a_t1 = dm_mt1L*qdd(1);
Ry = Ry + mt1L*(- a_n1*cos(beta_mt1L - phi) - a_t1*sin(beta_mt1L - phi));    

%  HIP
%mass mh at LL from foot
a_n1 = dm_LL*qd(1)^2;  
a_t1 = dm_LL*qdd(1);
Ry = Ry + mh*(- a_n1*cos(beta_LL - phi) - a_t1*sin(beta_LL - phi)); 

% RIGHT LEG (swing leg) (distances from hip)
%mt1R at (c1R) from hip 
a_n2 = (c1R)*qd(2)^2;  
a_t2 = (c1R)*qdd(2);
Ry = Ry + mt1R*(- a_n1*cos(beta_LL - phi) - a_t1*sin(beta_LL - phi) ...      
                + a_n2*cos(q(2) - theta ) + a_t2*sin(q(2) - theta));

%ms1R at (c1R+b1R) from hip 
a_n2 = (c1R + b1R)*qd(2)^2;  
a_t2 = (c1R + b1R)*qdd(2);
Ry = Ry + ms1R*(- a_n1*cos(beta_LL - phi) - a_t1*sin(beta_LL - phi) ...        
                + a_n2*cos(q(2) - theta ) + a_t2*sin(q(2) - theta));


% Assigning to temporary variables
Rx2LD = Rx;
Ry2LD = Ry;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%  Combined Ground Reaction Forces     %%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Reaction in X on left heel
Rx = (Rx2LS + Rx2LD);

%Reaction in Y on left heel
Ry = (Ry2LS + Ry2LD);

end

