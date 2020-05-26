function [Rx, Ry] = Forces_RIGHT_2L(q, qd, qdd, parameters, FootData, r)
%This function computes the ground reaction forces for RIGHT stance

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



%Instantanous Reaction forces and moment matricies
Rx = 0;  Ry = 0;


%Stance Leg Masses Geometric Parameters:
[psi, psi2, d_a, phi, theta_a, d_a2, phi2, theta_a2, height] ...
    = gContact('R', FootData, theta, q(1), q(2), r, 0, rRb, 0, LR, LL);

%ms1R at (a1R) from foot
dm_ms1R = sqrt(a1R^2 + d_a^2 + 2*a1R*d_a*sin(theta_a));
beta_ms1R =-asin((a1R*sin(pi - theta_a + pi/2)) / dm_ms1R);
%ms2R at (a1R + b1R) from foot
dm_ms2R = sqrt((a1R + b1R)^2 + d_a^2 + 2*(a1R + b1R)*d_a*sin(theta_a));
beta_ms2R = -asin(((a1R + b1R)*sin(pi - theta_a + pi/2)) / dm_ms2R);
%mt2R at (lsR + c2R) from foot
dm_mt2R = sqrt((lsR + c2R)^2 + d_a^2 + 2*(lsR + c2R)*d_a*sin(theta_a));
beta_mt2R =-asin(((lsR + c2R)*sin(pi - theta_a + pi/2)) / dm_mt2R);
%mt1R at (lsR + c2R + b2R) from foot
dm_mt1R = sqrt((lsR + c2R + b2R)^2 + d_a^2 + 2*(lsR + c2R + b2R)*d_a*sin(theta_a));
beta_mt1R = -asin(((lsR + c2R + b2R)*sin(pi - theta_a + pi/2)) / dm_mt1R);

%hip mass (mh) at (LR) from foot
dm_LR = sqrt(LR^2 + d_a^2 + 2*LR*d_a*sin(theta_a));
beta_LR = -asin((LR*sin(pi - theta_a + pi/2))/dm_LR);


%=====================================================================
%======================  GROUND REACTION FORCES ======================
%=====================================================================
%Rx is along ramp, 
%Ry is perpendicular to ramp



%-------------------------------------------------
%#################### STATICS ####################
%-------------------------------------------------
%  ******Rx along the ramp********
%  Right Leg (leg with ground contact) (distances from ground)
Rx = Rx + ms1R*g*sin(theta);    %mass ms1R at a1R from foot
Rx = Rx + ms2R*g*sin(theta);    %mass ms2R at a1R+b1R from foot
Rx = Rx + mt2R*g*sin(theta);    %mass mt2R at lsR+c2R from foot
Rx = Rx + mt1R*g*sin(theta);    %mass mt1R at lsR+c2R+b2R from foot
%  Hip
Rx = Rx + mh*g*sin(theta); %mass mh at LR
% Right Leg (swing leg) (distances from hip)
Rx = Rx + mt1L*g*sin(theta);    %mass mt1L at a2L from hip
Rx = Rx + mt2L*g*sin(theta);    %mass mt2L at a2L+b2L from hip
Rx = Rx + ms2L*g*sin(theta);    %mass mt2L at ltL+c1L from hip
Rx = Rx + ms1L*g*sin(theta);    %mass mt1L at ltL+c1L+b1L from hip


%  ******Ry perpendicular to ramp********
%  Right Leg (leg with ground contact) (distances from ground)
Ry = Ry + ms1R*g*cos(theta);    %mass ms1R at a1R from foot
Ry = Ry + ms2R*g*cos(theta);    %mass ms2R at a1R+b1R from foot
Ry = Ry + mt2R*g*cos(theta);    %mass mt2R at lsR+c2R from foot
Ry = Ry + mt1R*g*cos(theta);    %mass mt1R at lsR+c2R+b2R from foot
%  Hip
Ry = Ry + mh*g*cos(theta); %mass mh at LR
% Left Leg (swing leg) (distances from hip)
Ry = Ry + mt1L*g*cos(theta);    %mass mt1L at a2L from hip
Ry = Ry + mt2L*g*cos(theta);    %mass mt2L at a2L+b2L from hip
Ry = Ry + ms2L*g*cos(theta);    %mass mt2L at ltL+c1L from hip
Ry = Ry + ms1L*g*cos(theta);    %mass mt1L at ltL+c1R+b1R from hip


Rx2RS = Rx;
Ry2RS = Ry;


Rx = 0;
Ry = 0;



%--------------------------------------------------
%#################### DYNAMICS ####################  
%--------------------------------------------------
%    Normal Acceleration = a_n = L*qd^2
%    Tangential Acceleration = a_t = L*qdd
%    Total Relative Acceleration of swing mass = 
%                 a_swingmass-ground = a_hip-ground + a_swingmass-hip


%  ******Rx along the ramp AND Ry perpendicular to ramp********
%  RIGHT LEG (leg with ground contact) (distances from ground)
%mass ms1R at a1R from foot
a_n1 = dm_ms1R*qd(1)^2;  
a_t1 = dm_ms1R*qdd(1); 
Rx = Rx + ms1R*(-a_n1*sin(beta_ms1R-phi) - a_t1*cos(beta_ms1R-phi));
Ry = Ry + ms1R*(-a_n1*cos(beta_ms1R-phi) - a_t1*sin(beta_ms1R-phi));  
%mass ms2R at a1R+b1R from foot
a_n1 = dm_mt1R*qd(1)^2;  
a_t1 = dm_mt1R*qdd(1);
Rx = Rx + ms2R*(-a_n1*sin(beta_ms2R-phi) - a_t1*cos(beta_ms2R-phi));
Ry = Ry + ms2R*(-a_n1*cos(beta_ms2R-phi) - a_t1*sin(beta_ms2R-phi));  
%mass mt2R at lsR+c2R from foot
a_n1 = dm_mt2R*qd(1)^2;  
a_t1 = dm_mt2R*qdd(1); 
Rx = Rx + mt2R*(-a_n1*sin(beta_mt2R-phi) - a_t1*cos(beta_mt2R-phi));
Ry = Ry + mt2R*(-a_n1*cos(beta_mt2R-phi) - a_t1*sin(beta_mt2R-phi));  
%mass mt1R at lsR+c2R+b2R from foot
a_n1 = dm_mt1R*qd(1)^2;  
a_t1 = dm_mt1R*qdd(1);
Rx = Rx + mt1R*(-a_n1*sin(beta_mt1R-phi) - a_t1*cos(beta_mt1R-phi));
Ry = Ry + mt1R*(-a_n1*cos(beta_mt1R-phi) - a_t1*sin(beta_mt1R-phi)); 

%  HIP
%mass mh at LR from foot
a_n_hip = dm_LR*qd(1)^2;  
a_t_hip = dm_LR*qdd(1);
Rx = Rx + mh*(-a_n_hip*sin(beta_LR-phi) - a_t_hip*cos(beta_LR-phi));
Ry = Ry + mh*(-a_n_hip*cos(beta_LR-phi) - a_t_hip*sin(beta_LR-phi)); 

% LEFT LEG (swing leg) 
%mt1L at (a2L) from hip 
a_n2 = (a2L)*qd(2)^2;  
a_t2 = (a2L)*qdd(2);
Rx = Rx + mt1L*(-a_n_hip*sin(beta_LR-phi) - a_t_hip*cos(beta_LR-phi) ...
                  + a_n2*sin(q(2)-theta)  +    a_t2*cos(q(2)-theta));    
Ry = Ry + mt1L*(-a_n_hip*cos(beta_LR-phi) - a_t_hip*sin(beta_LR-phi) ...      
                  + a_n2*cos(q(2)-theta)  +    a_t2*sin(q(2)-theta));
%mt2L at (a2L + b2L) from hip
a_n2 = (a2L + b2L)*qd(2)^2;  
a_t2 = (a2L + b2L)*qdd(2);
Rx = Rx + mt2L*(-a_n_hip*sin(beta_LR-phi) - a_t_hip*cos(beta_LR-phi) ...
                  + a_n2*sin(q(2)-theta)  +    a_t2*cos(q(2)-theta));                   
Ry = Ry + mt2L*(-a_n_hip*cos(beta_LR-phi) - a_t_hip*sin(beta_LR-phi) ...        
                  + a_n2*cos(q(2)-theta)  +    a_t2*sin(q(2)-theta));
                            
%ms2L at (ltL + c1L) from hip
a_n2 = (ltL + c1L)*qd(2)^2;  
a_t2 = (ltL + c1L)*qdd(2);
Rx = Rx + ms2L*(-a_n_hip*sin(beta_LR-phi) - a_t_hip*cos(beta_LR-phi) ...
                  + a_n2*sin(q(2)-theta)  +    a_t2*cos(q(2)-theta));                   
Ry = Ry + ms2L*(-a_n_hip*cos(beta_LR-phi) - a_t_hip*sin(beta_LR-phi) ...        
                  + a_n2*cos(q(2)-theta)  +    a_t2*sin(q(2)-theta));

%ms1L at (ltL + c1L + b1L) from hip
a_n2 = (ltL + c1L + b1L)*qd(2)^2;  
a_t2 = (ltL + c1L + b1L)*qdd(2);
Rx = Rx + ms1L*(-a_n_hip*sin(beta_LR-phi) - a_t_hip*cos(beta_LR-phi) ...
                  + a_n2*sin(q(2)-theta)  +    a_t2*cos(q(2)-theta));                   
Ry = Ry + ms1L*(-a_n_hip*cos(beta_LR-phi) - a_t_hip*sin(beta_LR-phi) ...        
                  + a_n2*cos(q(2)-theta)  +    a_t2*sin(q(2)-theta));



              

Rx2RD = Rx;
Ry2RD = Ry;

Rx = 0;
Ry = 0;


%Reaction in X on left heel
Rx = (Rx2RS + Rx2RD);  %taking negative to mirror

%Reaction in Y on left heel
Ry = (Ry2RS + Ry2RD);


end

