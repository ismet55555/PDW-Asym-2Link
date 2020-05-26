%ENERGY OF MASSES
%FOR TREE LINK
%RIGHT STANCE

function [E, KE, PE] = Energy_RIGHT_3L(q, qd, parameters, FootData, rR)

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

rRb = FootData(2,2);

q1 = q(1);      q2 = q(2);      q3 = q(3);
q1d = qd(1);    q2d = qd(2);    q3d = qd(3);


%Define Distances:
[psi, psi2, d_a, phi, theta_a, d_a2, phi2, theta_a2, height] ...
    = gContact('R', FootData, theta, q1, q2, rR, 0, 0, 0, LR, LL);

%----------------  Stance Foot Distances --------------------------
%ms1R at (a1R)
dm_a1R = sqrt(a1R^2 + d_a^2 + 2*a1R*d_a*sin(theta_a));
beta_a1R = -asin((a1R*sin(pi - theta_a + pi/2))/dm_a1R);
psi_a1R = -(beta_a1R - phi + theta);
%ms2R at (a1R + b1R)
dm_a1Rb1R = sqrt((a1R + b1R)^2 + d_a^2 + 2*(a1R + b1R)*d_a*sin(theta_a));
beta_a1Rb1R = -asin(((a1R + b1R)*sin(pi - theta_a + pi/2))/dm_a1Rb1R);
psi_a1Rb1R = -(beta_a1Rb1R - phi + theta);

%mt2R at (a1R + b1R + c1R + c2R)
dm_lsRc2R = sqrt((a1R + b1R + c1R + c2R) ^2 + d_a^2 + 2*(a1R + b1R + c1R + c2R) *d_a*sin(theta_a));
beta_lsRc2R = -asin(((a1R + b1R + c1R + c2R) *sin(pi - theta_a + pi/2))/dm_lsRc2R);
psi_lsRc2R = -(beta_lsRc2R - phi + theta);
%mt1R at (a1R + b1R + c1R + c2R + b2R)
dm_lsRc2Rb2R = sqrt((a1R + b1R + c1R + c2R + b2R)  ^2 + d_a^2 + 2*(a1R + b1R + c1R + c2R + b2R)  *d_a*sin(theta_a));
beta_lsRc2Rb2R = -asin(((a1R + b1R + c1R + c2R + b2R)  *sin(pi - theta_a + pi/2))/dm_lsRc2Rb2R);
psi_lsRc2Rb2R = -(beta_lsRc2Rb2R - phi + theta);

%mh at (LR)
dm_LR = sqrt(LR^2 + d_a^2 + 2*LR*d_a*sin(theta_a));
beta_LR = -asin((LR*sin(pi - theta_a + pi/2))/dm_LR);
psi_LR = -(beta_LR - phi + theta);



%--------------------  Velocities --------------------
%Rolling of stance foot:
v_roll = [-rR*q1d*cos(pi/2 - psi - theta), rR*q1d*sin(pi/2 - psi - theta), 0];
v_roll = [0, 0, 0];

%Right (stance) Leg:
v_ms1R = [-dm_a1R*q1d*cos(psi_a1R),             -dm_a1R*q1d*sin(psi_a1R)            ];  %ms1R (lower shank)
v_ms2R = [-dm_a1Rb1R*q1d*cos(psi_a1Rb1R),       -dm_a1Rb1R*q1d*sin(psi_a1Rb1R)      ];  %ms2R (upper shank)
v_mt2R = [-dm_lsRc2R*q1d*cos(psi_lsRc2R),       -dm_lsRc2R*q1d*sin(psi_lsRc2R)      ];  %mt2R (lower thigh)
v_mt1R = [-dm_lsRc2Rb2R*q1d*cos(psi_lsRc2Rb2R), -dm_lsRc2Rb2R*q1d*sin(psi_lsRc2Rb2R)];  %mt1R (upper thigh)
v_mh   = [-dm_LR*q1d*cos(psi_LR),               -dm_LR*q1d*sin(psi_LR)              ];  %mh - Hip mass

%Left Thigh (swing) Leg:
v_mt1L = [(a2L)*q2d*cos(q2)     - dm_LR*q1d*cos(psi_LR), ... %mt1L (upper thigh)
          (a2L)*q2d*sin(q2)     - dm_LR*q1d*sin(psi_LR)];
v_mt2L = [(a2L+b2L)*q2d*cos(q2) - dm_LR*q1d*cos(psi_LR), ... %mt2L (lower thigh)
          (a2L+b2L)*q2d*sin(q2) - dm_LR*q1d*sin(psi_LR)];

%Left Shank (swing) Leg:
v_ms2L = [(c1L)*q3d*cos(q3)     + (ltL)*q2d*cos(q2)     - dm_LR*q1d*cos(psi_LR), ... %ms2L (upper shank)
          (c1L)*q3d*sin(q3)     + (ltL)*q2d*sin(q2)     - dm_LR*q1d*sin(psi_LR)];
v_ms1L = [(c1L+b1L)*q3d*cos(q3) + (ltL)*q2d*cos(q2)     - dm_LR*q1d*cos(psi_LR), ... %ms1L (lower shank)
          (c1L+b1L)*q3d*sin(q3) + (ltL)*q2d*sin(q2)     - dm_LR*q1d*sin(psi_LR)];



%-------------------- Velocity Magnitudes --------------------
v_mh_mag   = sqrt( v_mh(1)^2   + v_mh(2)^2 );   %-  hip
v_ms1R_mag = sqrt( v_ms1R(1)^2 + v_ms1R(2)^2 ); %\
v_ms2R_mag = sqrt( v_ms2R(1)^2 + v_ms2R(2)^2 ); % \
v_mt2R_mag = sqrt( v_mt2R(1)^2 + v_mt2R(2)^2 ); % / right (stance)
v_mt1R_mag = sqrt( v_mt1R(1)^2 + v_mt1R(2)^2 ); %/

v_mt1L_mag = sqrt( v_mt1L(1)^2 + v_mt1L(2)^2 );%\ left thigh(swing)
v_mt2L_mag = sqrt( v_mt2L(1)^2 + v_mt2L(2)^2 );%/

v_ms2L_mag = sqrt( v_ms2L(1)^2 + v_ms2L(2)^2 );%\ left shank(swing)
v_ms1L_mag = sqrt( v_ms1L(1)^2 + v_ms1L(2)^2 );%/




%--------------------  Kinetic Energy --------------------
KE =     0.5*mh*(v_mh_mag^2);%-  hip

KE = KE + 0.5*ms1R*(v_ms1R_mag^2);%\
KE = KE + 0.5*ms2R*(v_ms2R_mag^2);% \
KE = KE + 0.5*mt2R*(v_mt2R_mag^2);% / right (stance)
KE = KE + 0.5*mt1R*(v_mt1R_mag^2);%/

KE = KE + 0.5*mt1L*(v_mt1L_mag^2);%\
KE = KE + 0.5*mt2L*(v_mt2L_mag^2);%/ left thigh (swing)

KE = KE + 0.5*ms2L*(v_ms2L_mag^2);%\
KE = KE + 0.5*ms1L*(v_ms1L_mag^2);%/ left shank (swing)



%------------------  Postential Energy -------------------
PE =     mh*g*   (dm_LR*cos(psi_LR));                                    % -  hip

PE = PE + ms1R*g* (dm_a1R*cos(psi_a1R));                                  %\
PE = PE + ms2R*g* (dm_a1Rb1R*cos(psi_a1Rb1R));                            % \
PE = PE + mt2R*g* (dm_lsRc2R*cos(psi_lsRc2R));                            % / right (stance)
PE = PE + mt1R*g* (dm_lsRc2Rb2R*cos(psi_lsRc2Rb2R));                      %/

PE = PE + mt1L*g* (dm_LR*cos(psi_LR) - (a2L)*cos(q2));                    %\ left thigh(stance)
PE = PE + mt2L*g* (dm_LR*cos(psi_LR) - (a2L+b2L)*cos(q2));                %/

PE = PE + ms2L*g* (dm_LR*cos(psi_LR) - (ltL)*cos(q2) - (c1L)*cos(q3));    %\ left thigh(stance)
PE = PE + ms1L*g* (dm_LR*cos(psi_LR) - (ltL)*cos(q2) - (c1L+b1L)*cos(q3));%/





%Total Energy
E = KE + PE;

end