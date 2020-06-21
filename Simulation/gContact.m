function [psi, psi2, d_a, phi, theta_a, d_a2, phi2, theta_a2, height] ...
    = gContact(SIDE, p, q1, q2, rST, rSW, drdqST, drdqSW, LST, LSW)
% This function calculaes contact point of walker's feet and hip height.
% It also calculates the general walker gerometry
% NOTE:
%    "ST" = Stance Side
%    "SW" = Swing Side


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%    Foot Shape Parameters for Stance and Swing Foot    %%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% If Left foot is stance
if (SIDE == 'L')
    %Stance LEFT
    rSTa = p.walker.left.rLa;
    rSTb = p.walker.left.rLb;
    dST  = p.walker.left.dL;
    
    % Swing RIGHT
    rSWa = p.walker.right.rRa;
    rSWb = p.walker.right.rRb;
    dSW  = p.walker.right.dR;
end

% If Right foot is stance
if (SIDE == 'R')
    %Stance RIGHT
    rSWa = p.walker.left.rLa;
    rSWb = p.walker.left.rLb;
    dSW  = p.walker.left.dL;
    
    %Swing LEFT
    rSTa = p.walker.right.rRa;
    rSTb = p.walker.right.rRb;
    dST  = p.walker.right.dR;
    
end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%    Stance Foot Variables    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Foot Curve Derivative
%drdqST = rSTb;        %drdq0 = 0;  

%Angle between radial and tangent
psi = atan2(rST, drdqST);    

%New radius corresponding to ground contact point (approx)
rST = rSTa + rSTb*(q1 + p.sim.theta + (pi/2 - psi)); 

%Distance from foot angle to ground contact:
d_a = sqrt(dST^2 + rST^2 - 2*dST*rST*cos(  -(pi/2 - psi) + pi/2 - q1 - p.sim.theta  ));

%Angle at ground contact made by dST offset
phi = asin(  dST*sin(  -(pi/2 - psi) + pi/2 - q1 - p.sim.theta ) / d_a ) ;

%Angle between foot top and d_a
theta_a = pi - phi - (  -(pi/2 - psi) + pi/2 - q1 - p.sim.theta) ;

% Height of walker
height = abs(LST*cos(q1) + d_a*sin( theta_a - q1 ));




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%    Swing Foot Variables    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%drdqSW = rSWb;       %drdq0 = 0;
psi2 = atan2(rSW, drdqSW);
rSW = rSWa + rSWb*(q2 + p.sim.theta + (pi/2 - psi2)); 
d_a2 = sqrt(dSW^2 + rSW^2 - 2*dSW*rSW*cos(  -(pi/2 - psi2) + pi/2 - q2 - p.sim.theta  ));
phi2 = asin(  dSW*sin(  -(pi/2 - psi2) + pi/2 - q2 - p.sim.theta ) / d_a2 ) ;
theta_a2 = pi - phi2 - (  -(pi/2 - psi2) + pi/2 - q2  - p.sim.theta);




