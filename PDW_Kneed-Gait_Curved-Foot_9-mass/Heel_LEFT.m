function [Qm2, Qp2] = Heel_LEFT(q1, q1d, q2, q2d, FootData, rL, rR, parameters, psi)
%This fucntion calculates HEEL STRIKE AT THE END OF LEFT STANCE

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
 
%For Right Leg masses
%ms1R at (a1R)
dm_a1R = sqrt(a1R^2 + d_a2^2 + 2*a1R*d_a2*sin(theta_a2));                    
beta_a1R = -asin((a1R*sin(pi - theta_a2 + pi/2))/dm_a1R);
psi_a1R = -(beta_a1R - phi2 + theta);
%ms2R at (a1R + b1R)
dm_a1Rb1R = sqrt((a1R + b1R)^2 + d_a2^2 + 2*(a1R + b1R)*d_a2*sin(theta_a2));                    
beta_a1Rb1R = -asin(((a1R + b1R)*sin(pi - theta_a2 + pi/2))/dm_a1Rb1R);
psi_a1Rb1R = -(beta_a1Rb1R - phi2 + theta);

%mt2R at (a1R + b1R + c1R + c2R) 
dm_lsRc2R = sqrt((a1R + b1R + c1R + c2R) ^2 + d_a2^2 + 2*(a1R + b1R + c1R + c2R) *d_a2*sin(theta_a2)); 
beta_lsRc2R = -asin(((a1R + b1R + c1R + c2R) *sin(pi - theta_a2 + pi/2))/dm_lsRc2R);
psi_lsRc2R = -(beta_lsRc2R - phi2 + theta);

%mt1R at (a1R + b1R + c1R + c2R + b2R) 
dm_lsRc2Rb2R = sqrt((a1R + b1R + c1R + c2R + b2R)  ^2 + d_a2^2 + 2*(a1R + b1R + c1R + c2R + b2R)  *d_a2*sin(theta_a2)); 
beta_lsRc2Rb2R = -asin(((a1R + b1R + c1R + c2R + b2R)  *sin(pi - theta_a2 + pi/2))/dm_lsRc2Rb2R);
psi_lsRc2Rb2R = -(beta_lsRc2Rb2R - phi2 + theta);

%mh at (LR)
dm_LR = sqrt(LR^2 + d_a2^2 + 2*LR*d_a2*sin(theta_a2));                    
beta_LR = -asin((LR*sin(pi - theta_a2 + pi/2))/dm_LR);
psi_LR = -(beta_LR - phi2 + theta);


%=====================  Pre-Collision (m)  ===========================
Qm11 = dm_LL*dm_LR*mh*cos(psi_LL - psi_LR) - a2L*dm_lsLc2Lb2L*mt1L*cos(psi_lsLc2Lb2L - q1) - b1L*dm_a1L*ms1L*cos(psi_a1L - q1) - b2L*dm_lsLc2L*mt2L*cos(psi_lsLc2L - q1) - c1L*dm_a1L*ms1L*cos(psi_a1L - q1) - c1L*dm_a1Lb1L*ms2L*cos(psi_a1Lb1L - q1) - a2L*dm_lsLc2L*mt2L*cos(psi_lsLc2L - q1) + dm_LR*dm_a1L*ms1L*cos(psi_LR - psi_a1L) + dm_LL*dm_a1R*ms1R*cos(psi_LL - psi_a1R) + dm_LR*dm_a1Lb1L*ms2L*cos(psi_LR - psi_a1Lb1L) + dm_LL*dm_a1Rb1R*ms2R*cos(psi_LL - psi_a1Rb1R) + dm_LR*dm_lsLc2L*mt2L*cos(psi_LR - psi_lsLc2L) + dm_LL*dm_lsRc2R*mt2R*cos(psi_LL - psi_lsRc2R) + dm_LR*dm_lsLc2Lb2L*mt1L*cos(psi_LR - psi_lsLc2Lb2L) + dm_LL*dm_lsRc2Rb2R*mt1R*cos(psi_LL - psi_lsRc2Rb2R) - dm_a1L*ltL*ms1L*cos(psi_a1L - q1) - dm_a1Lb1L*ltL*ms2L*cos(psi_a1Lb1L - q1);
Qm12 = - dm_lsRc2R*mt2R*cos(psi_lsRc2R - q2)*(a2R + b2R) - dm_a1Rb1R*ms2R*cos(psi_a1Rb1R - q2)*(c1R + ltR) - a2R*dm_lsRc2Rb2R*mt1R*cos(psi_lsRc2Rb2R - q2) - dm_a1R*ms1R*cos(psi_a1R - q2)*(b1R + c1R + ltR);
Qm21 = - dm_lsLc2L*mt2L*cos(psi_lsLc2L - q1)*(a2L + b2L) - dm_a1Lb1L*ms2L*cos(psi_a1Lb1L - q1)*(c1L + ltL) - a2L*dm_lsLc2Lb2L*mt1L*cos(psi_lsLc2Lb2L - q1) - dm_a1L*ms1L*cos(psi_a1L - q1)*(b1L + c1L + ltL);
Qm22 = 0;

Qm2=[Qm11, Qm12; Qm21, Qm22];




%=====================  Post-Collision (p)  ======================
Qp11 = dm_LR^2*mh + dm_LR^2*ms1L + dm_LR^2*ms2L + dm_a1R^2*ms1R + dm_a1Rb1R^2*ms2R + dm_LR^2*mt1L + dm_LR^2*mt2L + dm_lsRc2R^2*mt2R + dm_lsRc2Rb2R^2*mt1R - a2L*dm_LR*mt1L*cos(psi_LR - q1) - a2L*dm_LR*mt2L*cos(psi_LR - q1) - b1L*dm_LR*ms1L*cos(psi_LR - q1) - b2L*dm_LR*mt2L*cos(psi_LR - q1) - c1L*dm_LR*ms1L*cos(psi_LR - q1) - c1L*dm_LR*ms2L*cos(psi_LR - q1) - dm_LR*ltL*ms1L*cos(psi_LR - q1) - dm_LR*ltL*ms2L*cos(psi_LR - q1);
Qp12 = a2L^2*mt1L + a2L^2*mt2L + b1L^2*ms1L + b2L^2*mt2L + c1L^2*ms1L + c1L^2*ms2L + ltL^2*ms1L + ltL^2*ms2L + 2*a2L*b2L*mt2L + 2*b1L*c1L*ms1L + 2*b1L*ltL*ms1L + 2*c1L*ltL*ms1L + 2*c1L*ltL*ms2L - a2L*dm_LR*mt1L*cos(psi_LR - q1) - a2L*dm_LR*mt2L*cos(psi_LR - q1) - b1L*dm_LR*ms1L*cos(psi_LR - q1) - b2L*dm_LR*mt2L*cos(psi_LR - q1) - c1L*dm_LR*ms1L*cos(psi_LR - q1) - c1L*dm_LR*ms2L*cos(psi_LR - q1) - dm_LR*ltL*ms1L*cos(psi_LR - q1) - dm_LR*ltL*ms2L*cos(psi_LR - q1);
Qp21 = -dm_LR*cos(psi_LR - q1)*(a2L*mt1L + a2L*mt2L + b1L*ms1L + b2L*mt2L + c1L*ms1L + c1L*ms2L + ltL*ms1L + ltL*ms2L);
Qp22 = mt2L*(a2L + b2L)^2 + ms2L*(c1L + ltL)^2 + a2L^2*mt1L + ms1L*(b1L + c1L + ltL)^2;

Qp2 = [Qp11, Qp12; Qp21, Qp22];



end

