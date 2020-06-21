%=========================================================================================

% Passive Dynamic Walker (PDW) Simulation Model
% 2-Link (Compass Gait) Curved Foot Variation

%=========================================================================================
%
% Walker Kinetic and Potential Energy Verification Tool
%
%=========================================================================================
%
% This script takes pre-recorded dynamics of a 2-Link 
% (compass gait) walker and inputs it into a set 
% of derived kinetic and potential energy equqations.
%
% This is done so that energy equations can be verified
% Theoretically, the total walker energy stays constant
%
% "Pre-recorded" vales are generated using a simplistic 2-link (compass gait) point 
% foot PDW model. A stable gait was obtained and a step was saved to text files.
%
%=========================================================================================
% Author: Ismet Handzic
% Date  : 10-23-2012
%=========================================================================================


clc			% Clear the command window
close all	% Close all open figures and plots
clear all   % Clear all variables in the workspace


%Walker Ramp Angle (rad)
theta = asin( 30 / 144);
theta = 0;

%Gravity (m/s^2)
g = 9.81;

% Stance Leg Mass (kg) 
mh   = 1.00;
mt1L = 0.00000001;
ms1L = 0.00000001;
mt1R = 0.00000001;
ms1R = 0.00000001;

% Mass Distribution Distances (m)
LL  = 1;
a1L = 0.33;
b1L = 0.33;

LR = 1;
a1R = 0.33;
b1R = 0.33;

% Foot Radius and ankle offset(m)
r = 1;
d = 0.5;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%    Walker Leg Angular Positions    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ------------------------------  Pre-Recorded Values ------------------------------------

% Load the pre-recorded angle ranges from file
q_data_3L  = dlmread('generated_test_data_q13L.txt');
q_data_2L  = dlmread('generated_test_data_q12L.txt');

% Pre-Recorded angle range for stance leg 
q13L = q_data_3L(:,1);
q12L = q_data_2L(:,1);
q1 = -[q13L; ...
       q12L]; 

% Pre-Recorded angle range for swing leg
q23L = q_data_3L(:,2);
q22L = q_data_2L(:,2);
q2   = [q23L; ...
        q22L]; 

% ------------------------------  Custom Range Values ------------------------------------
% Generated pivoting angle range for stance leg
q1_begin =   pi/2 + theta
q1_end   = -(pi/2- theta)
q1       = linspace(q1_begin, q1_end, length(q1)); 

% Generated pivoting angle range for swing leg
q2_begin = 3*pi/2 - theta
q2_end   =   pi/2- theta
q2 = linspace(q2_begin, q2_end, length(q2));


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%    Walker Leg Angular Velocities    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ------------------------------  Pre-Recorded Values ------------------------------------

% Load the pre-recorded angle ranges from file
qd_data_3L = dlmread('generated_test_data_qd13L.txt');
qd_data_2L = dlmread('generated_test_data_qd12L.txt');

% Pre-Recorded angle range for stance leg 
q1d3L = qd_data_3L(:,1);
q1d2L = qd_data_2L(:,1);
q1d   = -[q1d3L; q1d2L]; 

% Pre-Recorded angle range for swing leg
q2d3L = qd_data_3L(:,2);
q2d2L = qd_data_2L(:,2);
q2d   = [q2d3L; q2d2L]; 

% ------------------------------  Custom Range Values ------------------------------------
% Generated pivoting angle range for stance leg (Constant Angular Velocity)
q1d = ones(length(q1d),1)*-1;

% Generated pivoting angle range for swing leg (Constant Angular Velocity)
q2d = ones(length(q2d),1)* 1;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%    Pre-Allocating Vector Variables    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
T1 		= zeros(1,length(q1));
T2 		= zeros(1,length(q1));
T_Tot 	= zeros(1,length(q1));
ndT1 	= zeros(1,length(q1));
ndT2 	= zeros(1,length(q1));

V1 		= zeros(1,length(q1));
V2 		= zeros(1,length(q1));
V_Tot 	= zeros(1,length(q1));
ndV1 	= zeros(1,length(q1));
ndV2 	= zeros(1,length(q1));


rad_func = [r];

for i = 1 : length(q1) - 1
    % Foot Radius (m)
    r = 1.0;% + 0.6*q1(i);
	
	% Log foot Radius for this walker orientation
    rad_func = [rad_func; r];
	
	
	% FIXME: Double-check the curved foot distances and angles, see PDW code
    
	
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   Stance Masses    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	% Define distances and angles for curved foot
    theta_a = (pi/2 - q1(i)) + theta/2;
    phi 	= asin( ( d*sin(theta_a) ) / r);
    tau 	= pi - theta_a - phi;
    d_a 	= sqrt(r^2 + d^2 - 2*r*d*cos(tau));

    % Hip mass (mh)
    d_mh    = sqrt(d_a^2 + LL^2 - 2*d_a*LL*cos(theta_a + pi/2));
    beta_LL = asin( ( LL*sin(theta_a + pi/2) ) / d_mh);
    
    %mt1L at (a1L + b1L) from heel
    d_mt1L    = sqrt((a1L + b1L)^2 + d_a^2 + 2*(a1L + b1L)*d_a*sin(theta_a));
    beta_mt1L = -asin(((a1L + b1L)*sin(pi - theta_a + pi/2))/d_mt1L);

    %ms1L at (a1L)
    d_ms1L    =  sqrt(a1L^2 + d_a^2 + 2*a1L*d_a*sin(theta_a));
    beta_ms1L = -asin((a1L*sin(pi - theta_a + pi/2))/d_ms1L);
      

	% Kinetic Energy for stance masses
    T1(i) = 0.5* mh   * ( (d_mh   * q1d(i))^2) + ...
            0.5* mt1L * ( (d_mt1L * q1d(i))^2) + ...
            0.5* ms1L * ( (d_ms1L * q1d(i))^2);
			
	% Potential Energy for stance masses
    V1(i) = mh*g   *( d_mh1 *cos(beta_LL - phi)*cos(theta)) + ...
            mt1L*g *( d_mt1L*cos(beta_LL - phi)*cos(theta)) + ...
            ms1L*g *( d_ms1L*cos(beta_LL - phi)*cos(theta));
    

	
	
    
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   Swing Masses    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	% Define distances and angles for curved foot
    d_LL = sqrt(LL^2 + d_a^2 + 2*LL*d_a*sin(theta_a));
    psi  = asin(d_a*cos(theta_a) / d_LL);
    
    %mt1R at (a1R + b1R) from heel 
    dist = LR - a1R - b1R;
    v_12_mt1R =  sqrt(d_LL^2*q1d(i)^2 + (dist)^2*q2d(i)^2 ...
        + 2*d_LL*q1d(i)*(dist)*q2d(i)*cos(psi+q1(i)-q2(i)));
    d_amt1R = sqrt(LL^2 + (dist)^2 - 2*LL*(dist)*cos(-q1(i)+q2(i)));
    epsilon_mt1R = asin(((dist)*sin(-q1(i)+q2(i))) / d_amt1R);
    d_mt1R = sqrt(d_a^2 + d_amt1R^2 + 2* d_a * d_amt1R * sin(epsilon_mt1R + theta_a));
    omega_mt1R = asin( (d_amt1R*cos(epsilon_mt1R + theta_a)) / d_mt1R);
    
    
    %ms1R at (a1R) from heel 
    dist = LR - a1R;
    v_12_ms1R =  sqrt(d_LL^2*q1d(i)^2 + (dist)^2*q2d(i)^2 ...
               + 2*d_LL*q1d(i)*(dist)*q2d(i)*cos(psi+q1(i)-q2(i)));
    d_ams1R = sqrt(LL^2 + (dist)^2 - 2*LL*(dist)*cos(-q1(i)+q2(i)));
    epsilon_ms1R = asin(((dist)*sin(-q1(i)+q2(i))) / d_ams1R);
    d_ms1R = sqrt(d_a^2 + d_ams1R^2 + 2* d_a * d_ams1R * sin(epsilon_ms1R + theta_a));
    omega_ms1R = asin( (d_ams1R*cos(epsilon_ms1R + theta_a)) / d_ms1R);
    

	% Kinetic Energy for stance masses
    T2(i) = 0.5 * mt1R * v_12_mt1R^2 + ...
            0.5 * ms1R * v_12_ms1R^2;

	% Potential Energy for stance masses
    V2(i) = mt1R*g*( (d_mt1R*cos(omega_mt1R-phi)*cos(theta)) - (r*q1(i)*sin(theta)) )  + ...
            ms1R*g*( (d_ms1R*cos(omega_ms1R-phi)*cos(theta)) - (r*q1(i)*sin(theta)) );
end


% Total energy of the entire walker system
E = T1 + T2 + V1 + V2


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%    Numerical Derivative of Energies    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i = 1 : length(q1) - 1
    %Kinetic and Potential Energy of Stance Leg Masses
    ndT1(i) = (T1(i+1) - T1(i)) / (q1(i+1) - q1(i)); 
    ndV1(i) = (V1(i+1) - V1(i)) / (q1(i+1) - q1(i)); 

    %Kinetic and Potential Energy of Swing Leg Masses
    ndT2(i) = (T2(i+1) - T2(i)) / (q1(i+1) - q1(i)); 
    ndV2(i) = (V2(i+1) - V2(i)) / (q1(i+1) - q1(i)); 
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    Plots    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Kinetic and Potential Energies
fig001 = figure;
hold on
% Stance Masses
plot(q1, T1,'--r', 'Linewidth',3) 
plot(q1, V1,'-r', 'Linewidth',4)

% Swing Masses
plot(q1, T2,'--b', 'Linewidth',3) 
plot(q1, V2,'-b', 'Linewidth',3)

% Total Energy
plot(q1, E,'-k', 'Linewidth',3) 

title({"Potential and Kinetic Energy", "Curved Foot"})
legend('Stance - KE','Stance - PE','Swing - KE','Swing - PE', 'Total Energy')
xlabel('Stance Leg Angle (rad)')
ylabel('Energy (J)')
set(gca,'xLim',[min(q1),max(q1)]);
grid on
set(gcf,'position',[709   471   740   525])


%Numerical Derivative of Energies
fig002 = figure;
hold on
% Stance Masses
plot(q1,ndT1,'--r', 'Linewidth',2)
plot(q1,ndV1,'-r', 'Linewidth',2)

% Swing Masses
plot(q1,ndT2,'--b', 'Linewidth',2)
plot(q1,ndV2,'-b', 'Linewidth',2)

title({"Numerical Derivative of Energies", "Curved Foot"})
legend('Stance - KE','Stance - PE','Swing - KE','Swing - PE')
xlabel('Stance Leg Angle (rad)')
ylabel('Energy Derivative (J/rad)')
set(gca,'xLim',[min(q1),max(q1)]);
grid on
set(gcf,'position',[0   545   684   451])

