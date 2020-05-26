function [Rx, Ry] = Forces_LEFT_2L(q, qd, qdd, parameters, FootData, r)
%This function computes the ground reaction forces for LEFT stance

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
    = gContact('L', FootData, theta, q(1), q(2), r, 0, rLb, 0, LL, LR);


%ms1L at (a1L) from foot
dm_ms1L = sqrt(a1L^2 + d_a^2 + 2*a1L*d_a*sin(theta_a));
beta_ms1L =-asin((a1L*sin(pi - theta_a + pi/2)) / dm_ms1L);
%ms2L at (a1L + b1L) from foot
dm_ms2L = sqrt((a1L + b1L)^2 + d_a^2 + 2*(a1L + b1L)*d_a*sin(theta_a));
beta_ms2L = -asin(((a1L + b1L)*sin(pi - theta_a + pi/2)) / dm_ms2L);
%mt2L at (lsL + c2L) from foot
dm_mt2L = sqrt((lsL + c2L)^2 + d_a^2 + 2*(lsL + c2L)*d_a*sin(theta_a));
beta_mt2L =-asin(((lsL + c2L)*sin(pi - theta_a + pi/2)) / dm_mt2L);
%mt1L at (lsL + c2L + b2L) from foot
dm_mt1L = sqrt((lsL + c2L + b2L)^2 + d_a^2 + 2*(lsL + c2L + b2L)*d_a*sin(theta_a));
beta_mt1L = -asin(((lsL + c2L + b2L)*sin(pi - theta_a + pi/2)) / dm_mt1L);

%hip mass (mh) at (LL) from foot
dm_LL = sqrt(LL^2 + d_a^2 + 2*LL*d_a*sin(theta_a));
beta_LL = -asin((LL*sin(pi - theta_a + pi/2))/dm_LL);



%=====================================================================
%======================  GROUND REACTION FORCES ======================
%=====================================================================
%Rx is along ramp, 
%Ry is perpendicular to ramp

%-------------------------------------------------
%#################### STATICS ####################
%-------------------------------------------------
%  ******Rx along the ramp********
%  Left Leg (leg with ground contact) (distances from ground)
Rx = Rx + ms1L*g*sin(theta);    %mass ms1L at a1L from foot
Rx = Rx + ms2L*g*sin(theta);    %mass ms2L at a1L+b1L from foot
Rx = Rx + mt2L*g*sin(theta);    %mass mt2L at lsL+c2L from foot
Rx = Rx + mt1L*g*sin(theta);    %mass mt1L at lsL+c2L+b2L from foot
%  Hip
Rx = Rx + mh*g*sin(theta);      %mass mh at LL
% Right Leg (swing leg) (distances from hip)
Rx = Rx + mt1R*g*sin(theta);    %mass mt1R at a2R from hip
Rx = Rx + mt2R*g*sin(theta);    %mass mt2R at a2R+b2R from hip
Rx = Rx + ms2R*g*sin(theta);    %mass mt2R at ltR+c1R from hip
Rx = Rx + ms1R*g*sin(theta);    %mass mt1R at ltR+c1R+b1R from hip


%  ******Ry perpendicular to ramp********
%  Left Leg (leg with ground contact) (distances from ground)
Ry = Ry + ms1L*g*cos(theta);    %mass ms1L at a1L from foot
Ry = Ry + ms2L*g*cos(theta);    %mass ms2L at a1L+b1L from foot
Ry = Ry + mt2L*g*cos(theta);    %mass mt2L at lsL+c2L from foot
Ry = Ry + mt1L*g*cos(theta);    %mass mt1L at lsL+c2L+b2L from foot
%  Hip
Ry = Ry + mh*g*cos(theta);      %mass mh at LL
% Right Leg (swing leg) (distances from hip)
Ry = Ry + mt1R*g*cos(theta);    %mass mt1R at a2R from hip
Ry = Ry + mt2R*g*cos(theta);    %mass mt2R at a2R+b2R from hip
Ry = Ry + ms2R*g*cos(theta);    %mass mt2R at ltR+c1R from hip
Ry = Ry + ms1R*g*cos(theta);    %mass mt1R at ltR+c1R+b1R from hip


Rx2LS = Rx;
Ry2LS = Ry;

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
%  LEFT LEG (leg with ground contact) (distances from ground)
%mass ms1L at a1L from foot
a_n1 = dm_ms1L*qd(1)^2;  
a_t1 = dm_ms1L*qdd(1); 
Rx = Rx + ms1L*(-a_n1*sin(beta_ms1L-phi) - a_t1*cos(beta_ms1L-phi));
Ry = Ry + ms1L*(-a_n1*cos(beta_ms1L-phi) - a_t1*sin(beta_ms1L-phi));  
%mass ms2L at a1L+b1L from foot
a_n1 = dm_ms2L*qd(1)^2;  
a_t1 = dm_ms2L*qdd(1);
Rx = Rx + ms2L*(-a_n1*sin(beta_ms2L-phi) - a_t1*cos(beta_ms2L-phi));
Ry = Ry + ms2L*(-a_n1*cos(beta_ms2L-phi) - a_t1*sin(beta_ms2L-phi));  
%mass mt2L at lsL+c2L from foot
a_n1 = dm_mt2L*qd(1)^2;  
a_t1 = dm_mt2L*qdd(1); 
Rx = Rx + mt2L*(-a_n1*sin(beta_mt2L-phi) - a_t1*cos(beta_mt2L-phi));
Ry = Ry + mt2L*(-a_n1*cos(beta_mt2L-phi) - a_t1*sin(beta_mt2L-phi));  
%mass mt1L at lsL+c2L+b2L from foot
a_n1 = dm_mt1L*qd(1)^2;  
a_t1 = dm_mt1L*qdd(1);
Rx = Rx + mt1L*(-a_n1*sin(beta_mt1L-phi) - a_t1*cos(beta_mt1L-phi));
Ry = Ry + mt1L*(-a_n1*cos(beta_mt1L-phi) - a_t1*sin(beta_mt1L-phi)); 

%  HIP
%mass mh at LL from foot
a_n_hip = dm_LL*qd(1)^2;  
a_t_hip = dm_LL*qdd(1);
Rx = Rx + mh*(-a_n_hip*sin(beta_LL-phi) - a_t_hip*cos(beta_LL-phi));
Ry = Ry + mh*(-a_n_hip*cos(beta_LL-phi) - a_t_hip*sin(beta_LL-phi)); 

% RIGHT LEG (swing leg) 
%mt1R at (a2R) from hip 
a_n2 = (a2R)*qd(2)^2;  
a_t2 = (a2R)*qdd(2);
Rx = Rx + mt1R*(-a_n_hip*sin(beta_LL-phi) - a_t_hip*cos(beta_LL-phi) ...
                  + a_n2*sin(q(2)-theta)  +    a_t2*cos(q(2)-theta));    
Ry = Ry + mt1R*(-a_n_hip*cos(beta_LL-phi) - a_t_hip*sin(beta_LL-phi) ...      
                  + a_n2*cos(q(2)-theta)  + a_t2*sin(q(2)-theta));
%mt2R at (a2R + b2R) from hip
a_n2 = (a2R + b2R)*qd(2)^2;  
a_t2 = (a2R + b2R)*qdd(2);
Rx = Rx + mt2R*(-a_n_hip*sin(beta_LL-phi) - a_t_hip*cos(beta_LL-phi) ...
                  + a_n2*sin(q(2)-theta)  +    a_t2*cos(q(2)-theta));                   
Ry = Ry + mt2R*(-a_n_hip*cos(beta_LL-phi) - a_t_hip*sin(beta_LL-phi) ...        
                  + a_n2*cos(q(2)-theta)  +    a_t2*sin(q(2)-theta));
              
              
              
%ms2R at (ltR + c1R) from hip
a_n2 = (ltR + c1R)*qd(2)^2;  
a_t2 = (ltR + c1R)*qdd(2);
Rx = Rx + ms2R*(-a_n_hip*sin(beta_LL-phi) - a_t_hip*cos(beta_LL-phi) ...
                  + a_n2*sin(q(2)-theta)  +    a_t2*cos(q(2)-theta));                   
Ry = Ry + ms2R*(-a_n_hip*cos(beta_LL-phi) - a_t_hip*sin(beta_LL-phi) ...        
                  + a_n2*cos(q(2)-theta)  +    a_t2*sin(q(2)-theta));

%ms1R at (ltR + c1R + b1R) from hip
a_n2 = (ltR + c1R + b1R)*qd(2)^2;  
a_t2 = (ltR + c1R + b1R)*qdd(2);
Rx = Rx + ms1R*(-a_n_hip*sin(beta_LL-phi) - a_t_hip*cos(beta_LL-phi) ...
                  + a_n2*sin(q(2)-theta)  +    a_t2*cos(q(2)-theta));                   
Ry = Ry + ms1R*(-a_n_hip*cos(beta_LL-phi) - a_t_hip*sin(beta_LL-phi) ...        
                  + a_n2*cos(q(2)-theta)  +    a_t2*sin(q(2)-theta));



Rx2LD = Rx;
Ry2LD = Ry;

Rx = 0;
Ry = 0;


%Reaction in X on left heel
Rx = (Rx2LS + Rx2LD);  %taking negative to mirror

%Reaction in Y on left heel
Ry = (Ry2LS + Ry2LD);


end

