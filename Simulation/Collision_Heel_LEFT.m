function [Qm, Qp] = Collision_Heel_LEFT(q1, q1d, q2, q2d, p, rL, rR, psi)
%This fucntion calculates HEEL STRIKE AT THE END OF LEFT STANCE
% NOTE:
%    Pre and post collision matrix entries (ie. Qm12) are generated using generator
%    script.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%    Assigning all Incoming Parameters    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
   = gContact('L', p, q1, q2, rL, rR, rLb, rRb, LL, LR);


%For Left Leg masses
%ms1L at (a1L)
dm_a1L          =   sqrt(a1L^2 + d_a^2 - 2*a1L*d_a*cos(pi/2 + theta_a));    
beta_a1L        =   asin((a1L*sin(theta_a + pi/2))/dm_a1L);
psi_a1L         =   -(beta_a1L-phi+theta);
%mt1L at (a1L + b1L) 
dm_a1Lb1L       =   sqrt((a1L + b1L)^2 + d_a^2 - 2*(a1L + b1L)*d_a*cos(pi/2 + theta_a));
beta_a1Lb1L     =   asin(((a1L + b1L)*sin(theta_a + pi/2))/dm_a1Lb1L);
psi_a1Lb1L      =   -(beta_a1Lb1L-phi+theta);
%mh
dm_LL           =   sqrt(LL^2 + d_a^2 - 2*LL*d_a*cos(pi/2 + theta_a));                  
beta_LL         =   asin((LL*sin(theta_a + pi/2))/dm_LL);
psi_LL          =   -(beta_LL-phi+theta);
 
%For Right Leg masses
%ms1R at (a1R)
dm_a1R          =   sqrt(a1R^2 + d_a2^2 - 2*a1R*d_a2*cos(pi/2 + theta_a2));                   
beta_a1R        =   asin((a1R*sin(theta_a2 + pi/2))/dm_a1R);
psi_a1R         =   -(beta_a1R-phi2+theta);
%mt1R at (a1R + b1R) 
dm_a1Rb1R       =   sqrt((a1R + b1R)^2 + d_a2^2 - 2*(a1R + b1R)*d_a2*cos(pi/2 + theta_a2)); 
beta_a1Rb1R     =   asin(((a1R + b1R)*sin(theta_a2 + pi/2))/dm_a1Rb1R);
psi_a1Rb1R      =   -(beta_a1Rb1R-phi2+theta);
%mh
dm_LR           =   sqrt(LR^2 + d_a2^2 - 2*LR*d_a2*cos(pi/2 + theta_a2))   ;         
beta_LR         =   asin((LR*sin(theta_a2 + pi/2))/dm_LR);
psi_LR          =   -(beta_LR-phi2+theta);



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    Collision Matrix    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  Pre-Collision ("m")   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Qm11 = dm_LL*dm_LR*mh*cos(psi_LL - psi_LR) - c1L*dm_a1L*ms1L*cos(psi_a1L - q1) - c1L*dm_a1Lb1L*mt1L*cos(psi_a1Lb1L - q1) - b1L*dm_a1L*ms1L*cos(psi_a1L - q1) + dm_LR*dm_a1L*ms1L*cos(psi_LR - psi_a1L) + dm_LL*dm_a1R*ms1R*cos(psi_LL - psi_a1R) + dm_LR*dm_a1Lb1L*mt1L*cos(psi_LR - psi_a1Lb1L) + dm_LL*dm_a1Rb1R*mt1R*cos(psi_LL - psi_a1Rb1R);
Qm12 = - dm_a1R*ms1R*cos(psi_a1R - q2)*(b1R + c1R) - c1R*dm_a1Rb1R*mt1R*cos(psi_a1Rb1R - q2);
Qm21 = - dm_a1L*ms1L*cos(psi_a1L - q1)*(b1L + c1L) - c1L*dm_a1Lb1L*mt1L*cos(psi_a1Lb1L - q1);
Qm22 = 0;

% Combining the whole pre-collision matrix
Qm = [Qm11, Qm12;
      Qm21, Qm22];


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    Collision Matrix    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  Post-Collision ("m")   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Qp11 = dm_LR^2*mh + dm_LR^2*ms1L + dm_a1R^2*ms1R + dm_LR^2*mt1L + dm_a1Rb1R^2*mt1R - b1L*dm_LR*ms1L*cos(psi_LR - q1) - c1L*dm_LR*ms1L*cos(psi_LR - q1) - c1L*dm_LR*mt1L*cos(psi_LR - q1);
Qp12 = b1L^2*ms1L + c1L^2*ms1L + c1L^2*mt1L + 2*b1L*c1L*ms1L - b1L*dm_LR*ms1L*cos(psi_LR - q1) - c1L*dm_LR*ms1L*cos(psi_LR - q1) - c1L*dm_LR*mt1L*cos(psi_LR - q1);
Qp21 = -dm_LR*cos(psi_LR - q1)*(b1L*ms1L + c1L*ms1L + c1L*mt1L);
Qp22 = ms1L*(b1L + c1L)^2 + c1L^2*mt1L;

% Combining the whole post-collision matrix
Qp = [Qp11, Qp12;
      Qp21, Qp22];

end

