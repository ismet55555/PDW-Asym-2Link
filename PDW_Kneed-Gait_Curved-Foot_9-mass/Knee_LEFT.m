function [Qm, Qp] = Knee_LEFT(q1, q1d, q2, q2d, q3, q3d, FootData, rL, rR, parameters, psi)
%This fucntion calculates KNEE STRIKE AT THE END OF 3-LINK LEFT STANCE

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

rLa = FootData(1,1);    rLb = FootData(1,2);   dL = FootData(1,3);
rRa = FootData(2,1);    rRb = FootData(2,2);   dR = FootData(2,3);



%Define Distances from ground contact
[psi, psi2, d_a, phi, theta_a, d_a2, phi2, theta_a2, height] ...
   = gContact('L', FootData, theta, q1, q2, rL, rR, rLb, rRb, LL, LR);


%For Left Leg masses
%ms1L at (a1L)
dm_a1L = sqrt(a1L^2 + d_a^2 + 2*a1L*d_a*sin(theta_a));                    
beta_a1L = -asin((a1L*sin(pi - theta_a + pi/2))/dm_a1L);
psi_a1L = -(beta_a1L - phi + theta);
%ms2L at (a1L + b1L)
dm_a1Lb1L = sqrt((a1L + b1L)^2 + d_a^2 + 2*(a1L + b1L)*d_a*sin(theta_a));                    
beta_a1Lb1L = -asin(((a1L + b1L)*sin(pi - theta_a + pi/2))/dm_a1Lb1L);
psi_a1Lb1L = -(beta_a1Lb1L - phi + theta);

%mt2L at (a1L + b1L + c1L + c2L) 
dm_lsLc2L = sqrt((a1L + b1L + c1L + c2L) ^2 + d_a^2 + 2*(a1L + b1L + c1L + c2L) *d_a*sin(theta_a)); 
beta_lsLc2L = -asin(((a1L + b1L + c1L + c2L) *sin(pi - theta_a + pi/2))/dm_lsLc2L);
psi_lsLc2L = -(beta_lsLc2L - phi + theta);

%mt1L at (a1L + b1L + c1L + c2L + b2L) 
dm_lsLc2Lb2L = sqrt((a1L + b1L + c1L + c2L + b2L)  ^2 + d_a^2 + 2*(a1L + b1L + c1L + c2L + b2L)  *d_a*sin(theta_a)); 
beta_lsLc2Lb2L = -asin(((a1L + b1L + c1L + c2L + b2L)  *sin(pi - theta_a + pi/2))/dm_lsLc2Lb2L);
psi_lsLc2Lb2L = -(beta_lsLc2Lb2L - phi + theta);

%mh at (LL)
dm_LL = sqrt(LL^2 + d_a^2 + 2*LL*d_a*sin(theta_a));                    
beta_LL = -asin((LL*sin(pi - theta_a + pi/2))/dm_LL);
psi_LL = -(beta_LL - phi + theta);
 




%=====================  Pre-Collision (m)  ===========================
Qm11 = dm_LL^2*mh + dm_LL^2*ms1R + dm_LL^2*ms2R + dm_a1L^2*ms1L + dm_a1Lb1L^2*ms2L + dm_LL^2*mt1R + dm_LL^2*mt2R + dm_lsLc2L^2*mt2L + dm_lsLc2Lb2L^2*mt1L - a2R*dm_LL*mt1R*cos(psi_LL - q2) - a2R*dm_LL*mt2R*cos(psi_LL - q2) - b1R*dm_LL*ms1R*cos(psi_LL - q3) - b2R*dm_LL*mt2R*cos(psi_LL - q2) - c1R*dm_LL*ms1R*cos(psi_LL - q3) - c1R*dm_LL*ms2R*cos(psi_LL - q3) - dm_LL*ltR*ms1R*cos(psi_LL - q2) - dm_LL*ltR*ms2R*cos(psi_LL - q2);
Qm12 = a2R^2*mt1R + a2R^2*mt2R + b2R^2*mt2R + ltR^2*ms1R + ltR^2*ms2R + 2*a2R*b2R*mt2R - a2R*dm_LL*mt1R*cos(psi_LL - q2) - a2R*dm_LL*mt2R*cos(psi_LL - q2) - b2R*dm_LL*mt2R*cos(psi_LL - q2) + b1R*ltR*ms1R*cos(q2 - q3) + c1R*ltR*ms1R*cos(q2 - q3) + c1R*ltR*ms2R*cos(q2 - q3) - dm_LL*ltR*ms1R*cos(psi_LL - q2) - dm_LL*ltR*ms2R*cos(psi_LL - q2);
Qm13 = b1R^2*ms1R + c1R^2*ms1R + c1R^2*ms2R + 2*b1R*c1R*ms1R - b1R*dm_LL*ms1R*cos(psi_LL - q3) - c1R*dm_LL*ms1R*cos(psi_LL - q3) - c1R*dm_LL*ms2R*cos(psi_LL - q3) + b1R*ltR*ms1R*cos(q2 - q3) + c1R*ltR*ms1R*cos(q2 - q3) + c1R*ltR*ms2R*cos(q2 - q3);
Qm21 = -dm_LL*(a2R*mt1R*cos(psi_LL - q2) + a2R*mt2R*cos(psi_LL - q2) + b1R*ms1R*cos(psi_LL - q3) + b2R*mt2R*cos(psi_LL - q2) + c1R*ms1R*cos(psi_LL - q3) + c1R*ms2R*cos(psi_LL - q3) + ltR*ms1R*cos(psi_LL - q2) + ltR*ms2R*cos(psi_LL - q2));
Qm22 = a2R^2*mt1R + a2R^2*mt2R + b2R^2*mt2R + ltR^2*ms1R + ltR^2*ms2R + 2*a2R*b2R*mt2R + b1R*ltR*ms1R*cos(q2 - q3) + c1R*ltR*ms1R*cos(q2 - q3) + c1R*ltR*ms2R*cos(q2 - q3);
Qm23 = b1R^2*ms1R + c1R^2*ms1R + c1R^2*ms2R + 2*b1R*c1R*ms1R + b1R*ltR*ms1R*cos(q2 - q3) + c1R*ltR*ms1R*cos(q2 - q3) + c1R*ltR*ms2R*cos(q2 - q3);

Qm = [Qm11, Qm12, Qm13; ... 
      Qm21, Qm22, Qm23];




%=====================  Post-Collision (p)  ======================
Qp11 = dm_LL^2*mh + dm_LL^2*ms1R + dm_LL^2*ms2R + dm_a1L^2*ms1L + dm_a1Lb1L^2*ms2L + dm_LL^2*mt1R + dm_LL^2*mt2R + dm_lsLc2L^2*mt2L + dm_lsLc2Lb2L^2*mt1L - a2R*dm_LL*mt1R*cos(psi_LL - q2) - a2R*dm_LL*mt2R*cos(psi_LL - q2) - b1R*dm_LL*ms1R*cos(psi_LL - q2) - b2R*dm_LL*mt2R*cos(psi_LL - q2) - c1R*dm_LL*ms1R*cos(psi_LL - q2) - c1R*dm_LL*ms2R*cos(psi_LL - q2) - dm_LL*ltR*ms1R*cos(psi_LL - q2) - dm_LL*ltR*ms2R*cos(psi_LL - q2);
Qp12 = a2R^2*mt1R + a2R^2*mt2R + b1R^2*ms1R + b2R^2*mt2R + c1R^2*ms1R + c1R^2*ms2R + ltR^2*ms1R + ltR^2*ms2R + 2*a2R*b2R*mt2R + 2*b1R*c1R*ms1R + 2*b1R*ltR*ms1R + 2*c1R*ltR*ms1R + 2*c1R*ltR*ms2R - a2R*dm_LL*mt1R*cos(psi_LL - q2) - a2R*dm_LL*mt2R*cos(psi_LL - q2) - b1R*dm_LL*ms1R*cos(psi_LL - q2) - b2R*dm_LL*mt2R*cos(psi_LL - q2) - c1R*dm_LL*ms1R*cos(psi_LL - q2) - c1R*dm_LL*ms2R*cos(psi_LL - q2) - dm_LL*ltR*ms1R*cos(psi_LL - q2) - dm_LL*ltR*ms2R*cos(psi_LL - q2);
Qp21 = -dm_LL*cos(psi_LL - q2)*(a2R*mt1R + a2R*mt2R + b1R*ms1R + b2R*mt2R + c1R*ms1R + c1R*ms2R + ltR*ms1R + ltR*ms2R);
Qp22 = mt2R*(a2R + b2R)^2 + ms2R*(c1R + ltR)^2 + a2R^2*mt1R + ms1R*(b1R + c1R + ltR)^2;

 
Qp = [Qp11, Qp12; ...
      Qp21, Qp22];



end

