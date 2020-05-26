%ENERGY OF MASSES
%FOR TWO LINK
%LEFT STANCE

function [E, KE, PE] = Energy_LEFT_2L(q, qd, parameters, FootData, rL)

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


q1 = q(1);      q2 = q(2);
q1d = qd(1);    q2d = qd(2);

rLb = FootData(1,2);



%Define Distances:
[psi, psi2, d_a, phi, theta_a, d_a2, phi2, theta_a2, height] ...
            = gContact('L', FootData, theta, q1, 0, rL, 0, 0, 0, LL, 0);

%ms1L at (a1L) (lower shank)
dm_a1L = sqrt((a1L)^2 + d_a^2 + 2*(a1L)*d_a*sin(theta_a));
beta_a1L = -asin(((a1L)*sin(pi - theta_a + pi/2))/dm_a1L);
psi_a1L = -(beta_a1L - phi + theta);
%ms2L at (a1L + b1L) (upper shank)
dm_a1Lb1L = sqrt((a1L + b1L)^2 + d_a^2 + 2*(a1L + b1L)*d_a*sin(theta_a));
beta_a1Lb1L = -asin(((a1L + b1L)*sin(pi - theta_a + pi/2))/dm_a1Lb1L);
psi_a1Lb1L = -(beta_a1Lb1L - phi + theta);

%mt2L at (lsL + c2L) (lower thigh)
dm_lsLc2L = sqrt((lsL + c2L)^2 + d_a^2 + 2*(lsL + c2L)*d_a*sin(theta_a));
beta_lsLc2L = -asin(((lsL + c2L)*sin(pi - theta_a + pi/2))/dm_lsLc2L);
psi_lsLc2L = -(beta_lsLc2L - phi + theta);
%mt1L at (lsL + c2L + b2L) (upper thigh)
dm_lsLc2Lb2L = sqrt((lsL + c2L + b2L)^2 + d_a^2 + 2*(lsL + c2L + b2L)*d_a*sin(theta_a));
beta_lsLc2Lb2L = -asin(((lsL + c2L + b2L)*sin(pi - theta_a + pi/2))/dm_lsLc2Lb2L);
psi_lsLc2Lb2L = -(beta_lsLc2Lb2L - phi + theta);

%mh (hip mass)
dm_LL = sqrt(LL^2 + d_a^2 + 2*LL*d_a*sin(theta_a));
beta_LL = -asin((LL*sin(pi - theta_a + pi/2))/dm_LL);
psi_LL = -(beta_LL - phi + theta);




%--------------------  Velocities --------------------
%Rolling of stance foot:
v_roll = [-rL*q1d*cos(pi/2 - psi - theta),...
           rL*q1d*sin(pi/2 - psi - theta), 0];
v_roll = [0, 0, 0];

%Left (stance) Leg:
v_ms1L = [-dm_a1L*q1d*cos(psi_a1L),            -dm_a1L*q1d*sin(psi_a1L)            ];  %ms1L (lower shank)
v_ms2L = [-dm_a1Lb1L*q1d*cos(psi_a1Lb1L),      -dm_a1Lb1L*q1d*sin(psi_a1Lb1L)      ];  %ms2L (upper shank)
v_mt2L = [-dm_lsLc2L*q1d*cos(psi_lsLc2L),      -dm_lsLc2L*q1d*sin(psi_lsLc2L)      ];  %mt2L (lower thigh)
v_mt1L = [-dm_lsLc2Lb2L*q1d*cos(psi_lsLc2Lb2L),-dm_lsLc2Lb2L*q1d*sin(psi_lsLc2Lb2L)];  %mt1L (upper thigh)

%mh - Hip mass
v_mh   = [-dm_LL*q1d*cos(psi_LL),              -dm_LL*q1d*sin(psi_LL)              ];

%Right (swing) Leg:
v_mt1R = [(a2R)*q2d*cos(q2)     - dm_LL*q1d*cos(psi_LL),...        %mt1R (upper thigh)
          (a2R)*q2d*sin(q2)     - dm_LL*q1d*sin(psi_LL) ];
v_mt2R = [(a2R+b2R)*q2d*cos(q2) - dm_LL*q1d*cos(psi_LL),...         %mt2R (lower thigh)
          (a2R+b2R)*q2d*sin(q2) - dm_LL*q1d*sin(psi_LL)];

v_ms2R = [(ltR+c1R)*q2d*cos(q2)     - dm_LL*q1d*cos(psi_LL),...    %ms2R (upper shank)
          (ltR+c1R)*q2d*sin(q2)     - dm_LL*q1d*sin(psi_LL) ];
v_ms1R = [(ltR+c1R+b1R)*q2d*cos(q2) - dm_LL*q1d*cos(psi_LL),...     %ms1R (lower shank)
          (ltR+c1R+b1R)*q2d*sin(q2) - dm_LL*q1d*sin(psi_LL)];


%Velocity Magnitudes:
v_mh_mag   = sqrt( v_mh(1)^2   + v_mh(2)^2 );  %-  hip

v_ms1L_mag = sqrt( v_ms1L(1)^2 + v_ms1L(2)^2 );%\ left shank
v_ms2L_mag = sqrt( v_ms2L(1)^2 + v_ms2L(2)^2 );%/
v_mt2L_mag = sqrt( v_mt2L(1)^2 + v_mt2L(2)^2 );%\ left thigh
v_mt1L_mag = sqrt( v_mt1L(1)^2 + v_mt1L(2)^2 );%/

v_mt1R_mag = sqrt( v_mt1R(1)^2 + v_mt1R(2)^2 );%\ right thigh
v_mt2R_mag = sqrt( v_mt2R(1)^2 + v_mt2R(2)^2 );%/
v_ms2R_mag = sqrt( v_ms2R(1)^2 + v_ms2R(2)^2 );%\ right shank
v_ms1R_mag = sqrt( v_ms1R(1)^2 + v_ms1R(2)^2 );%/




%--------------------  Kinetic Energy --------------------
KE =     0.5*mh*(v_mh_mag^2);%-  hip

KE = KE + 0.5*ms1L*(v_ms1L_mag^2);%\ left shank
KE = KE + 0.5*ms2L*(v_ms2L_mag^2);%/
KE = KE + 0.5*mt2L*(v_mt2L_mag^2);%\ left thigh
KE = KE + 0.5*mt1L*(v_mt1L_mag^2);%/

KE = KE + 0.5*ms1R*(v_ms1R_mag^2);%\ right thigh
KE = KE + 0.5*ms2R*(v_ms2R_mag^2);%/
KE = KE + 0.5*mt2R*(v_mt2R_mag^2);%\ right shank
KE = KE + 0.5*mt1R*(v_mt1R_mag^2);%/




%------------------  Potential Energy -------------------
PE =     mh*g*   (dm_LL*cos(psi_LL));                        % -  hip

PE = PE + ms1L*g* (dm_a1L      *cos(psi_a1L));                %\ left shank
PE = PE + ms2L*g* (dm_a1Lb1L   *cos(psi_a1Lb1L));             %/
PE = PE + mt2L*g* (dm_lsLc2L   *cos(psi_lsLc2L));             %\ left thigh
PE = PE + mt1L*g* (dm_lsLc2Lb2L*cos(psi_lsLc2Lb2L));          %/

PE = PE + mt1R*g* (dm_LL*cos(psi_LL) - (a2R)        *cos(q2));%\ right thigh
PE = PE + mt2R*g* (dm_LL*cos(psi_LL) - (a2R+b2R)    *cos(q2));%/
PE = PE + ms2R*g* (dm_LL*cos(psi_LL) - (ltR+c1R)    *cos(q2));%\ right shank
PE = PE + ms1R*g* (dm_LL*cos(psi_LL) - (ltR+c1R+b1R)*cos(q2));%/



%Total Energy
E = KE + PE;

end