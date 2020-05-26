%Ismet Handzic
%REEDLab Univerisy of South Florida
%
%This file calculates contact point of walker's feet and hip height


function [psi, psi2, d_a, phi, theta_a, d_a2, phi2, theta_a2, height] ...
    = gContact(SIDE, FootData, theta, q1, q2, rST, rSW, drdqST, drdqSW, LST, LSW)

if (SIDE == 'L')
    %Stance LEFT and Swing RIGHT
    rSTa = FootData(1,1);    rSTb = FootData(1,2);  dST = FootData(1,3);
    rSWa = FootData(2,1);    rSWb = FootData(2,2);  dSW = FootData(2,3);
end
if (SIDE == 'R')
    %Stance RIGHT and Swing LEFT
    rSWa = FootData(1,1);    rSWb = FootData(1,2);  dSW = FootData(1,3);
    rSTa = FootData(2,1);    rSTb = FootData(2,2);  dST = FootData(2,3);
end



%=========================== Stance Foot ==================================

%Foot Curve Derivative Slope
%drdqST = rSTb;        %drdq0 = 0;  

%Angle between radial and tangent
psi = atan2(rST, drdqST);

%New radius corresponding to ground contact point (approx)
rST = rSTa + rSTb*(q1 + theta + (pi/2 - psi));

%Distance from foot angle to ground contact:
d_a = sqrt(dST^2 + rST^2 - 2*dST*rST*cos(  -(pi/2 - psi) + pi/2 - q1 - theta  ));

%Angle at ground contact made by dST offset
phi = asin(  dST*sin(  -(pi/2 - psi) + pi/2 - q1 - theta ) / d_a ) ;

%Angle between foot top and d_a
theta_a = pi - phi - (  -(pi/2 - psi) + pi/2 - q1 - theta) ;


%Hip Height
height = abs(LST*cos(q1) + d_a*sin( theta_a - q1 ));



%============================ Swing Foot ==================================

%drdqSW = rSWb;       %drdq0 = 0;
psi2 = atan2(rSW, drdqSW);
rSW = rSWa + rSWb*(q2 + theta + (pi/2 - psi2)); 
d_a2 = sqrt(dSW^2 + rSW^2 - 2*dSW*rSW*cos(  -(pi/2 - psi2) + pi/2 - q2 - theta  ));
phi2 = asin(  dSW*sin(  -(pi/2 - psi2) + pi/2 - q2 - theta ) / d_a2 ) ;
theta_a2 = pi - phi2 - (  -(pi/2 - psi2) + pi/2 - q2  - theta);


%for swinging shank, replace q2 with q3

