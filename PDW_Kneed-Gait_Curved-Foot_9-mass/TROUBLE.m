clear all
close all
clc


mt1R = 1;      mt1L = 1; 
mt2R = 2;      mt2L = 2; 
ms2R = 3;      ms2L = 3;
ms1R = 4;      ms1L = 4;
a1R  = 5;      a1L  = 5;
b1R  = 6;      b1L  = 6; 
c1R  = 7;      c1L  = 7;
c2R  = 8;      c2L  = 8;
b2R  = 9;      b2L  = 9;
a2R  = 10;     a2L  = 10;  
lsR  = 11;     lsL  = 11;
ltR  = 12;     ltL  = 12;
LR   = 13;     LL   = 13;

mh   = 3;

dm_LL           = LL;
dm_lsLc2Lb2L    = a1L+b1L+c1L+c2L+b2L;
dm_lsLc2L       = a1L+b1L+c1L+c2L;
dm_a1Lb1L       = a1L+b1L;
dm_a1L          = a1L;

q1 = 1;
q2 = 1;
q3 = 1;

q1d = 1;
q2d = 1;
q3d = 1;

psi_LL           = q1;
psi_lsLc2Lb2L    = q1;
psi_lsLc2L       = q1;
psi_a1Lb1L       = q1;
psi_a1L          = q1;

g     = 1;     
dt    = 1;    
theta = 1;

% syms a1L b1L c1L c2L b2L a2L  LL
% syms a1R b1R c1R c2R b2R a2R  LR
% 
% syms ms1L ms2L mt2L mt1L   mh   
% syms ms1R ms2R mt2R mt1R
% 
% syms  q1 q1d q1dd  g
% syms  q2 q2d q2dd
% syms  q3 q3d q3dd

rLa = 0;
rLb = 0;
rL  = 0;
dL  = 0.00000000000000000000000000000001; 

rRa = 0;
rRb = 0;
rR  = 0;
dR  = 0.00000000000000000000000000000001;

%Curved Foot
M12_C = - (mt1R*(2*a2R*sin(asin((dL*sin(pi/2 + q1 - theta))/((rLa + q1*rLb)^2 + dL^2 + dL*cos(pi/2 + q1 - theta)*(2*rLa + 2*q1*rLb))^(1/2)) - asin((LL*sin(asin((dL*sin(pi/2 + q1 - theta))/((rLa + q1*rLb)^2 + dL^2 + dL*cos(pi/2 + q1 - theta)*(2*rLa + 2*q1*rLb))^(1/2)) - theta - q1))/((rLa + q1*rLb)^2 + LL^2 + dL^2 - 2*LL*sin(asin((dL*sin(pi/2 + q1 - theta))/((rLa + q1*rLb)^2 + dL^2 + dL*cos(pi/2 + q1 - theta)*(2*rLa + 2*q1*rLb))^(1/2)) - q1 - theta - pi/2)*((rLa + q1*rLb)^2 + dL^2 + dL*cos(pi/2 + q1 - theta)*(2*rLa + 2*q1*rLb))^(1/2) + dL*cos(pi/2 + q1 - theta)*(2*rLa + 2*q1*rLb))^(1/2)) - theta)*sin(q2)*((rLa + q1*rLb)^2 + LL^2 + dL^2 - 2*LL*sin(asin((dL*sin(pi/2 + q1 - theta))/((rLa + q1*rLb)^2 + dL^2 + dL*cos(pi/2 + q1 - theta)*(2*rLa + 2*q1*rLb))^(1/2)) - q1 - theta - pi/2)*((rLa + q1*rLb)^2 + dL^2 + dL*cos(pi/2 + q1 - theta)*(2*rLa + 2*q1*rLb))^(1/2) + dL*cos(pi/2 + q1 - theta)*(2*rLa + 2*q1*rLb))^(1/2) + 2*a2R*cos(asin((dL*sin(pi/2 + q1 - theta))/((rLa + q1*rLb)^2 + dL^2 + dL*cos(pi/2 + q1 - theta)*(2*rLa + 2*q1*rLb))^(1/2)) - asin((LL*sin(asin((dL*sin(pi/2 + q1 - theta))/((rLa + q1*rLb)^2 + dL^2 + dL*cos(pi/2 + q1 - theta)*(2*rLa + 2*q1*rLb))^(1/2)) - theta - q1))/((rLa + q1*rLb)^2 + LL^2 + dL^2 - 2*LL*sin(asin((dL*sin(pi/2 + q1 - theta))/((rLa + q1*rLb)^2 + dL^2 + dL*cos(pi/2 + q1 - theta)*(2*rLa + 2*q1*rLb))^(1/2)) - q1 - theta - pi/2)*((rLa + q1*rLb)^2 + dL^2 + dL*cos(pi/2 + q1 - theta)*(2*rLa + 2*q1*rLb))^(1/2) + dL*cos(pi/2 + q1 - theta)*(2*rLa + 2*q1*rLb))^(1/2)) - theta)*cos(q2)*((rLa + q1*rLb)^2 + LL^2 + dL^2 - 2*LL*sin(asin((dL*sin(pi/2 + q1 - theta))/((rLa + q1*rLb)^2 + dL^2 + dL*cos(pi/2 + q1 - theta)*(2*rLa + 2*q1*rLb))^(1/2)) - q1 - theta - pi/2)*((rLa + q1*rLb)^2 + dL^2 + dL*cos(pi/2 + q1 - theta)*(2*rLa + 2*q1*rLb))^(1/2) + dL*cos(pi/2 + q1 - theta)*(2*rLa + 2*q1*rLb))^(1/2)))/2 - (ms1R*(2*ltR*sin(asin((dL*sin(pi/2 + q1 - theta))/((rLa + q1*rLb)^2 + dL^2 + dL*cos(pi/2 + q1 - theta)*(2*rLa + 2*q1*rLb))^(1/2)) - asin((LL*sin(asin((dL*sin(pi/2 + q1 - theta))/((rLa + q1*rLb)^2 + dL^2 + dL*cos(pi/2 + q1 - theta)*(2*rLa + 2*q1*rLb))^(1/2)) - theta - q1))/((rLa + q1*rLb)^2 + LL^2 + dL^2 - 2*LL*sin(asin((dL*sin(pi/2 + q1 - theta))/((rLa + q1*rLb)^2 + dL^2 + dL*cos(pi/2 + q1 - theta)*(2*rLa + 2*q1*rLb))^(1/2)) - q1 - theta - pi/2)*((rLa + q1*rLb)^2 + dL^2 + dL*cos(pi/2 + q1 - theta)*(2*rLa + 2*q1*rLb))^(1/2) + dL*cos(pi/2 + q1 - theta)*(2*rLa + 2*q1*rLb))^(1/2)) - theta)*sin(q2)*((rLa + q1*rLb)^2 + LL^2 + dL^2 - 2*LL*sin(asin((dL*sin(pi/2 + q1 - theta))/((rLa + q1*rLb)^2 + dL^2 + dL*cos(pi/2 + q1 - theta)*(2*rLa + 2*q1*rLb))^(1/2)) - q1 - theta - pi/2)*((rLa + q1*rLb)^2 + dL^2 + dL*cos(pi/2 + q1 - theta)*(2*rLa + 2*q1*rLb))^(1/2) + dL*cos(pi/2 + q1 - theta)*(2*rLa + 2*q1*rLb))^(1/2) + 2*ltR*cos(asin((dL*sin(pi/2 + q1 - theta))/((rLa + q1*rLb)^2 + dL^2 + dL*cos(pi/2 + q1 - theta)*(2*rLa + 2*q1*rLb))^(1/2)) - asin((LL*sin(asin((dL*sin(pi/2 + q1 - theta))/((rLa + q1*rLb)^2 + dL^2 + dL*cos(pi/2 + q1 - theta)*(2*rLa + 2*q1*rLb))^(1/2)) - theta - q1))/((rLa + q1*rLb)^2 + LL^2 + dL^2 - 2*LL*sin(asin((dL*sin(pi/2 + q1 - theta))/((rLa + q1*rLb)^2 + dL^2 + dL*cos(pi/2 + q1 - theta)*(2*rLa + 2*q1*rLb))^(1/2)) - q1 - theta - pi/2)*((rLa + q1*rLb)^2 + dL^2 + dL*cos(pi/2 + q1 - theta)*(2*rLa + 2*q1*rLb))^(1/2) + dL*cos(pi/2 + q1 - theta)*(2*rLa + 2*q1*rLb))^(1/2)) - theta)*cos(q2)*((rLa + q1*rLb)^2 + LL^2 + dL^2 - 2*LL*sin(asin((dL*sin(pi/2 + q1 - theta))/((rLa + q1*rLb)^2 + dL^2 + dL*cos(pi/2 + q1 - theta)*(2*rLa + 2*q1*rLb))^(1/2)) - q1 - theta - pi/2)*((rLa + q1*rLb)^2 + dL^2 + dL*cos(pi/2 + q1 - theta)*(2*rLa + 2*q1*rLb))^(1/2) + dL*cos(pi/2 + q1 - theta)*(2*rLa + 2*q1*rLb))^(1/2)))/2 - (ms2R*(2*ltR*sin(asin((dL*sin(pi/2 + q1 - theta))/((rLa + q1*rLb)^2 + dL^2 + dL*cos(pi/2 + q1 - theta)*(2*rLa + 2*q1*rLb))^(1/2)) - asin((LL*sin(asin((dL*sin(pi/2 + q1 - theta))/((rLa + q1*rLb)^2 + dL^2 + dL*cos(pi/2 + q1 - theta)*(2*rLa + 2*q1*rLb))^(1/2)) - theta - q1))/((rLa + q1*rLb)^2 + LL^2 + dL^2 - 2*LL*sin(asin((dL*sin(pi/2 + q1 - theta))/((rLa + q1*rLb)^2 + dL^2 + dL*cos(pi/2 + q1 - theta)*(2*rLa + 2*q1*rLb))^(1/2)) - q1 - theta - pi/2)*((rLa + q1*rLb)^2 + dL^2 + dL*cos(pi/2 + q1 - theta)*(2*rLa + 2*q1*rLb))^(1/2) + dL*cos(pi/2 + q1 - theta)*(2*rLa + 2*q1*rLb))^(1/2)) - theta)*sin(q2)*((rLa + q1*rLb)^2 + LL^2 + dL^2 - 2*LL*sin(asin((dL*sin(pi/2 + q1 - theta))/((rLa + q1*rLb)^2 + dL^2 + dL*cos(pi/2 + q1 - theta)*(2*rLa + 2*q1*rLb))^(1/2)) - q1 - theta - pi/2)*((rLa + q1*rLb)^2 + dL^2 + dL*cos(pi/2 + q1 - theta)*(2*rLa + 2*q1*rLb))^(1/2) + dL*cos(pi/2 + q1 - theta)*(2*rLa + 2*q1*rLb))^(1/2) + 2*ltR*cos(asin((dL*sin(pi/2 + q1 - theta))/((rLa + q1*rLb)^2 + dL^2 + dL*cos(pi/2 + q1 - theta)*(2*rLa + 2*q1*rLb))^(1/2)) - asin((LL*sin(asin((dL*sin(pi/2 + q1 - theta))/((rLa + q1*rLb)^2 + dL^2 + dL*cos(pi/2 + q1 - theta)*(2*rLa + 2*q1*rLb))^(1/2)) - theta - q1))/((rLa + q1*rLb)^2 + LL^2 + dL^2 - 2*LL*sin(asin((dL*sin(pi/2 + q1 - theta))/((rLa + q1*rLb)^2 + dL^2 + dL*cos(pi/2 + q1 - theta)*(2*rLa + 2*q1*rLb))^(1/2)) - q1 - theta - pi/2)*((rLa + q1*rLb)^2 + dL^2 + dL*cos(pi/2 + q1 - theta)*(2*rLa + 2*q1*rLb))^(1/2) + dL*cos(pi/2 + q1 - theta)*(2*rLa + 2*q1*rLb))^(1/2)) - theta)*cos(q2)*((rLa + q1*rLb)^2 + LL^2 + dL^2 - 2*LL*sin(asin((dL*sin(pi/2 + q1 - theta))/((rLa + q1*rLb)^2 + dL^2 + dL*cos(pi/2 + q1 - theta)*(2*rLa + 2*q1*rLb))^(1/2)) - q1 - theta - pi/2)*((rLa + q1*rLb)^2 + dL^2 + dL*cos(pi/2 + q1 - theta)*(2*rLa + 2*q1*rLb))^(1/2) + dL*cos(pi/2 + q1 - theta)*(2*rLa + 2*q1*rLb))^(1/2)))/2 - (mt2R*(2*cos(asin((dL*sin(pi/2 + q1 - theta))/((rLa + q1*rLb)^2 + dL^2 + dL*cos(pi/2 + q1 - theta)*(2*rLa + 2*q1*rLb))^(1/2)) - asin((LL*sin(asin((dL*sin(pi/2 + q1 - theta))/((rLa + q1*rLb)^2 + dL^2 + dL*cos(pi/2 + q1 - theta)*(2*rLa + 2*q1*rLb))^(1/2)) - theta - q1))/((rLa + q1*rLb)^2 + LL^2 + dL^2 - 2*LL*sin(asin((dL*sin(pi/2 + q1 - theta))/((rLa + q1*rLb)^2 + dL^2 + dL*cos(pi/2 + q1 - theta)*(2*rLa + 2*q1*rLb))^(1/2)) - q1 - theta - pi/2)*((rLa + q1*rLb)^2 + dL^2 + dL*cos(pi/2 + q1 - theta)*(2*rLa + 2*q1*rLb))^(1/2) + dL*cos(pi/2 + q1 - theta)*(2*rLa + 2*q1*rLb))^(1/2)) - theta)*cos(q2)*(a2R + b2R)*((rLa + q1*rLb)^2 + LL^2 + dL^2 - 2*LL*sin(asin((dL*sin(pi/2 + q1 - theta))/((rLa + q1*rLb)^2 + dL^2 + dL*cos(pi/2 + q1 - theta)*(2*rLa + 2*q1*rLb))^(1/2)) - q1 - theta - pi/2)*((rLa + q1*rLb)^2 + dL^2 + dL*cos(pi/2 + q1 - theta)*(2*rLa + 2*q1*rLb))^(1/2) + dL*cos(pi/2 + q1 - theta)*(2*rLa + 2*q1*rLb))^(1/2) + 2*sin(asin((dL*sin(pi/2 + q1 - theta))/((rLa + q1*rLb)^2 + dL^2 + dL*cos(pi/2 + q1 - theta)*(2*rLa + 2*q1*rLb))^(1/2)) - asin((LL*sin(asin((dL*sin(pi/2 + q1 - theta))/((rLa + q1*rLb)^2 + dL^2 + dL*cos(pi/2 + q1 - theta)*(2*rLa + 2*q1*rLb))^(1/2)) - theta - q1))/((rLa + q1*rLb)^2 + LL^2 + dL^2 - 2*LL*sin(asin((dL*sin(pi/2 + q1 - theta))/((rLa + q1*rLb)^2 + dL^2 + dL*cos(pi/2 + q1 - theta)*(2*rLa + 2*q1*rLb))^(1/2)) - q1 - theta - pi/2)*((rLa + q1*rLb)^2 + dL^2 + dL*cos(pi/2 + q1 - theta)*(2*rLa + 2*q1*rLb))^(1/2) + dL*cos(pi/2 + q1 - theta)*(2*rLa + 2*q1*rLb))^(1/2)) - theta)*sin(q2)*(a2R + b2R)*((rLa + q1*rLb)^2 + LL^2 + dL^2 - 2*LL*sin(asin((dL*sin(pi/2 + q1 - theta))/((rLa + q1*rLb)^2 + dL^2 + dL*cos(pi/2 + q1 - theta)*(2*rLa + 2*q1*rLb))^(1/2)) - q1 - theta - pi/2)*((rLa + q1*rLb)^2 + dL^2 + dL*cos(pi/2 + q1 - theta)*(2*rLa + 2*q1*rLb))^(1/2) + dL*cos(pi/2 + q1 - theta)*(2*rLa + 2*q1*rLb))^(1/2)))/2
%M12_C = simple(M12_C)

%Point Foot
M12_P = -LL*cos(q1 - q2)*(a2R*ms1R + a2R*ms2R + a2R*mt1R + a2R*mt2R + b2R*ms1R + b2R*ms2R + b2R*mt2R + c2R*ms1R + c2R*ms2R)

