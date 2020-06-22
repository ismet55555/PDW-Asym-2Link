%=========================================================================================
%
% Passive Dynamic Walker (PDW) Simulation Model
% 2-Link (Compass Gait) Curved Foot Variation
%
%=========================================================================================
%
% Walker Dynamics Matrices Generator/Tool
%
%=========================================================================================
%
% This script generates the inertia (M), velocity (N), and gravity (G), matrix 
% for a 2-link (compass gait) curved feet passive dynamic walker .
%
% The script will only generate a selected side, left or right. That is, the resulting 
% dynamics matrices will either be for the left or right stance phase of the walker
%
% The resulting matrices are eventually used to  compute the walkers instantaneous 
% leg angular accelerations (qdd). The angular accelerations are then numerically 
% integrated to obtain angular velocities (qd) and angular positions (q) of the walker's 
% legs.
%
%=========================================================================================
% Author: Ismet Handzic
% Date  : 12/25/2013
%=========================================================================================


clc			% Clear the command window
close all	% Close all open figures and plots
clear all   % Clear all variables in the workspace


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%    User Input for Left or Right Stance    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
user_input = input('Calculate LEFT or RIGHT Stance Dynamics? Enter "L" or "R"  : ', 's');


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%    LEFT Stance Walker Dynamics    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if (user_input == 'L') | (user_input == 'l')
    fprintf('\nComputing LEFT Stance/RIGHT Swing Dynamic Equations...\n\n')

	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   Define Symbolics    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    syms a1L b1L c1L LL
    syms a1R b1R c1R 
	
    syms dm_a1L  dm_a1Lb1L  dm_LL
    
    syms ms1L mt1L   mh   
    syms ms1R mt1R

	syms g theta
    syms q1 q1d q1dd
    syms q2 q2d q2dd
    
    syms psi_LL psi_a1L psi_a1Lb1L
    
    syms psi
    
    syms v_roll
    syms rLa rLb r3 r4
    syms dL
    
	
	% Define foot shape radius
    rL = rLa + rLb*q1;
    
    d_a     = sqrt(rL^2 + dL^2 + 2*rL*dL*cos(pi/2 + q1 - theta));
    phi     = asin( (dL*sin(pi/2 + q1 - theta)) / d_a);
    theta_a = pi/2 - phi + q1 + theta;
    
    %ms1L at (a1L)
    dm_a1L   = sqrt(a1L^2 + d_a^2 + 2*a1L*d_a*sin(theta_a));                    
    beta_a1L = -asin((a1L*sin(pi - theta_a + pi/2))/dm_a1L);
    psi_a1L  = -(beta_a1L-phi+theta);
	
    %mt1L at (a1L + b1L) 
    dm_a1Lb1L   = sqrt((a1L + b1L)^2 + d_a^2 + 2*(a1L + b1L)*d_a*sin(theta_a)); 
    beta_a1Lb1L = -asin(((a1L + b1L)*sin(pi - theta_a + pi/2))/dm_a1Lb1L);
    psi_a1Lb1L  = -(beta_a1Lb1L-phi+theta);
	
    %mh
    dm_LL   = sqrt(LL^2 + d_a^2 + 2*LL*d_a*sin(theta_a));                    
    beta_LL = -asin((LL*sin(pi - theta_a + pi/2))/dm_LL);
    psi_LL  = -(beta_LL-phi+theta);
    

	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   Mass Velocities    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %Left (stance) Leg:
    v_mh   = [-dm_LL*q1d*cos(psi_LL),          -dm_LL*q1d*sin(psi_LL)        ];  %mh - Hip mass
    v_ms1L = [-dm_a1L*q1d*cos(psi_a1L),        -dm_a1L*q1d*sin(psi_a1L)      ];  %mt1L (upper thigh)
    v_mt1L = [-dm_a1Lb1L*q1d*cos(psi_a1Lb1L),  -dm_a1Lb1L*q1d*sin(psi_a1Lb1L)];  %ms1L (lower shank)

    %Right (swing) Leg:
    v_mt1R = [(c1R)*q2d*cos(q2) - dm_LL*q1d*cos(psi_LL),...     
              (c1R)*q2d*sin(q2) - dm_LL*q1d*sin(psi_LL)    ];  %mt1R (upper thigh)
    v_ms1R = [(c1R+b1R)*q2d*cos(q2) - dm_LL*q1d*cos(psi_LL),...
              (c1R+b1R)*q2d*sin(q2) - dm_LL*q1d*sin(psi_LL)];  %ms1R (lower shank)


	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   Kinetic Energy    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %Velocity Magnitudes:
    v_mh_mag   = sqrt( v_mh(1)^2   + v_mh(2)^2 );    %-  hip
    v_ms1L_mag = sqrt( v_ms1L(1)^2 + v_ms1L(2)^2 );  %\ left (stance)
    v_mt1L_mag = sqrt( v_mt1L(1)^2 + v_mt1L(2)^2 );  %/

    v_mt1R_mag = sqrt( v_mt1R(1)^2 + v_mt1R(2)^2 );  %\ right 
    v_ms1R_mag = sqrt( v_ms1R(1)^2 + v_ms1R(2)^2 );  %/
    
    K =   0.5*mh*(v_mh_mag^2) ...      %-  hip
        + 0.5*ms1L*(v_ms1L_mag^2) ...  %\ left (stance)
        + 0.5*mt1L*(v_mt1L_mag^2) ...  %/
        + 0.5*mt1R*(v_mt1R_mag^2) ...  %\ right 
        + 0.5*ms1R*(v_ms1R_mag^2);     %/


	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   Potential Energy    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    P =   mh*g*   (dm_LL*cos(psi_LL)) ...                  % -  hip
        + ms1L*g* (dm_a1L*cos(psi_a1L)) ...                %\ left (stance)
        + mt1L*g* (dm_a1Lb1L*cos(psi_a1Lb1L)) ...          %/
        + mt1R*g* (dm_LL*cos(psi_LL) - (c1R)*cos(q2)) ...  %\ right 
        + ms1R*g* (dm_LL*cos(psi_LL) - (c1R+b1R)*cos(q2)); %/


	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   Lagrangian    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    L = K - P;

    Equations = Lagrange(L, [q1, q1d, q1dd,   q2, q2d, q2dd]);
    %if "Lagrange()" function not found, download at:
    %    http://www.mathworks.com/matlabcentral/fileexchange/23037-lagranges-equations/content/Lagrange.m


    
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   Inertia (M) Matrix    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    fprintf('------------------------------------------------\n')
    fprintf('-------------- Inertia Matrix (M) --------------\n')
    fprintf('------------------------------------------------')
    [M11, KM11] = coeffs(Equations(1), q1dd);
    M11         = (M11(1))
    
    [M12, KM12] = coeffs(Equations(1), q2dd);
    M12 		= (M12(1))
    
    [M21, KM21] = coeffs(Equations(2), q1dd);
    M21 		= (M21(1))
    
    [M22, KM22] = coeffs(Equations(2), q2dd);
    M22 		= (M22(1))
	
	% Combine entire matrix
    M 		 = [M11, M12; M21, M22]
	% Output text character length of each matrix element
    M_length = [length(char(M11)), length(char(M12)); length(char(M21)), length(char(M22))]

    
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   Velocity (N) Matrix    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    fprintf('\n\n-------------------------------------------------\n')
    fprintf('-------------- Velocity Matrix (N) --------------\n')
    fprintf('-------------------------------------------------\n')
    [N11, KN11] = coeffs(Equations(1), q1d);
    N11_qd_2    = N11(1)  %for qd^2
    N11_qd      = N11(2)  %for qd
    
    [N12, KN12] = coeffs(Equations(1), q2d);
    N12_qd_2    = N12(1)  %for qd^2
    N12_qd      = N12(2)  %for qd
    
    [N21, KN21] = coeffs(Equations(2), q1d);
    N21_qd_2    = (N21(1))  %for qd^2
    N21_qd      = (N21(1))  %for qd
    
    [N22, KN22] = coeffs(Equations(2), q2d);
    N22_qd_2    = (N22(1))  %for qd^2
    N22_qd      = (N22(1))  %for qd

	% Combine entire matrix
    N_qd_2 = [N11_qd_2,  N12_qd_2;
		      N21_qd_2,  0]
    N_qd   = [N11_qd, N12_qd;
		      0,      0]	
				   
	% Output text character length of each matrix element	   
    N_qd_2_length = [length(char(N11_qd_2)), length(char(N12_qd_2)); 
					 length(char(N21_qd_2)), 0]
    N_qd_length   = [length(char(N11_qd)), length(char(N12_qd)); 
					 0, 				   0]


	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   Gravity (G) Matrix    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    fprintf('\n\n-------------------------------------------------\n')
    fprintf('-------------- Gravity Matrix (G)  --------------\n')
    fprintf('-------------------------------------------------\n')
    [G11, KG11] = coeffs(Equations(1), g);
    G11         = (G11(1))*g
    
    [G21, KG21] = coeffs(Equations(2), g);
    G21         = (G21(1))*g
	
	% Combine entire matrix
    G 		 = [G11; 
				G21]

	% Output text character length of each matrix element
    G_length = [length(char(G11)); 
				length(char(G21))]
    
    
    
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%%%%%%   Saving Matrices to File    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	fprintf('Writing LEFT stance matrix data to file ... ')
	% Open File
    fid = fopen('2Link_Dynamics_Equations_LEFT.txt', 'w');
    
	% Write all Matrices to File
    fprintf(fid, 'M11 = %s;\n',  char(M11));
    fprintf(fid, 'M12 = %s;\n',  char(M12));
    fprintf(fid, 'M21 = M12;\n', char(M21));
    fprintf(fid, 'M22 = %s;\n',  char(M22));
    fprintf(fid, '\n\n');
    fprintf(fid, 'N11_qd_2 = %s;\n',  char(N11_qd_2));
    fprintf(fid, 'N12_qd_2 = %s;\n',  char(N12_qd_2));
    fprintf(fid, 'N21_qd_2 = %s;\n',  char(N21_qd_2));
    fprintf(fid, 'N22_qd_2 = 0;\n',   char(N22_qd_2));
    fprintf(fid, '\n\n');
    fprintf(fid, 'N11_qd = %s;\n',  char(N11_qd));
    fprintf(fid, 'N12_qd = %s;\n',  char(N12_qd));
    fprintf(fid, 'N21_qd = 0;\n',   char(N21_qd));
    fprintf(fid, 'N22_qd = 0;\n',   char(N22_qd));
    fprintf(fid, '\n\n');
    fprintf(fid, 'G11 = %s;\n',  char(G11));
    fprintf(fid, 'G21 = %s;\n',  char(G21));
    
	% Close File
    fclose(fid);
	fprintf('DONE\n\n')
end




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%    RIGHT Stance Walker Dynamics    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if (user_input == 'R') | (user_input == 'r')
    fprintf('\nComputing RIGHT Stance/Left Swing Dynamic Equations...\n\n')

    syms a1L b1L c1L 
    syms a1R b1R c1R LR
    
    syms dm_a1R  dm_a1Rb1R  dm_LR 
    
    syms ms1L mt1L   mh   
    syms ms1R mt1R
	
	syms g theta
    syms q1 q1d q1dd
    syms q2 q2d q2dd
    
    syms psi_LR psi_a1R psi_a1Rb1R
    
    syms psi
    
    syms v_roll
    syms rR rRa rRb
    syms dR
	
	% Define foot shape radius
    rR = rRa + rRb*q1;
    
    d_a = sqrt(rR^2 + dR^2 + 2*rR*dR*cos(pi/2 + q1 - theta));
    phi = asin( (dR*sin(pi/2 + q1 - theta)) / d_a);
    theta_a = pi/2 - phi + q1 + theta;
    
    %ms1R at (a1R)
    dm_a1R = sqrt(a1R^2 + d_a^2 + 2*a1R*d_a*sin(theta_a));                    
    beta_a1R = -asin((a1R*sin(pi - theta_a + pi/2))/dm_a1R);
    psi_a1R = -(beta_a1R-phi+theta);
	
    %mt1R at (a1R + b1R) 
    dm_a1Rb1R = sqrt((a1R + b1R)^2 + d_a^2 + 2*(a1R + b1R)*d_a*sin(theta_a)); 
    beta_a1Rb1R = -asin(((a1R + b1R)*sin(pi - theta_a + pi/2))/dm_a1Rb1R);
    psi_a1Rb1R = -(beta_a1Rb1R-phi+theta); 
	
    %mh
    dm_LR = sqrt(LR^2 + d_a^2 + 2*LR*d_a*sin(theta_a));                    
    beta_LR = -asin((LR*sin(pi - theta_a + pi/2))/dm_LR);
    psi_LR = -(beta_LR-phi+theta);
    
    

	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   Mass Velocities    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %Right (stance) Leg:
    v_mh   = [-dm_LR*q1d*cos(psi_LR),          -dm_LR*q1d*sin(psi_LR)        ];  %mh - Hip mass
    v_ms1R = [-dm_a1R*q1d*cos(psi_a1R),        -dm_a1R*q1d*sin(psi_a1R)      ];  %mt1R (upper thigh)
    v_mt1R = [-dm_a1Rb1R*q1d*cos(psi_a1Rb1R),  -dm_a1Rb1R*q1d*sin(psi_a1Rb1R)];  %ms1R (lower shank)
    
    %Left (swing) Leg:
    v_mt1L = [(c1L)*q2d*cos(q2) - dm_LR*q1d*cos(psi_LR),...     
              (c1L)*q2d*sin(q2) - dm_LR*q1d*sin(psi_LR)    ];  %mt1L (upper thigh)
    v_ms1L = [(c1L+b1L)*q2d*cos(q2) - dm_LR*q1d*cos(psi_LR),...
              (c1L+b1L)*q2d*sin(q2) - dm_LR*q1d*sin(psi_LR)];  %ms1L (lower shank) 
                 
    
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   Kinetic Energy    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %Velocity Magnitudes:
    v_mh_mag   = sqrt( v_mh(1)^2   + v_mh(2)^2 );    %-  hip
    v_ms1R_mag = sqrt( v_ms1R(1)^2 + v_ms1R(2)^2 );  %\ right (stance)
    v_mt1R_mag = sqrt( v_mt1R(1)^2 + v_mt1R(2)^2 );  %/

    v_mt1L_mag = sqrt( v_mt1L(1)^2 + v_mt1L(2)^2 );  %\ left 
    v_ms1L_mag = sqrt( v_ms1L(1)^2 + v_ms1L(2)^2 );  %/
        
    K =   0.5*mh*  (v_mh_mag^2) ...    %-  hip
        + 0.5*ms1R*(v_ms1R_mag^2) ...  %\ right (stance)
        + 0.5*mt1R*(v_mt1R_mag^2) ...  %/
        + 0.5*mt1L*(v_mt1L_mag^2) ...  %\ left 
        + 0.5*ms1L*(v_ms1L_mag^2);     %/


	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   Potential Energy    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    P =   mh*g*   (dm_LR*cos(psi_LR)) ...                   % -  hip
        + ms1R*g* (dm_a1R*cos(psi_a1R)) ...                 %\ right (stance)
        + mt1R*g* (dm_a1Rb1R*cos(psi_a1Rb1R)) ...           %/
        + mt1L*g* (dm_LR*cos(psi_LR) - (c1L)*cos(q2)) ...   %\ left 
        + ms1L*g* (dm_LR*cos(psi_LR) - (c1L+b1L)*cos(q2));  %/
    
    
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   Lagrangian    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    L = K - P;

    Equations = Lagrange(L, [q1, q1d, q1dd,   q2, q2d, q2dd]);
    %if "Lagrange()" function not found, download at:
    %    http://www.mathworks.com/matlabcentral/fileexchange/23037-lagranges-equations/content/Lagrange.m



	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   Inertia (M) Matrix    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %Inertia (M) Matrix
    fprintf('------------------------------------------------\n')
    fprintf('-------------- Inertia Matrix (M) --------------\n')
    fprintf('------------------------------------------------')
    [M11, KM11] = coeffs(Equations(1), q1dd);
    M11 		= (M11(1))
    
    [M12, KM12] = coeffs(Equations(1), q2dd);
    M12 		= (M12(1))
    
    [M21, KM21] = coeffs(Equations(2), q1dd);
    M21 		= (M21(1))
    
    [M22, KM22] = coeffs(Equations(2), q2dd);
    M22 		= (M22(1))

	% Combine entire matrix
    M 		 = [M11, M12; 
	            M21, M22]
	% Output text character length of each matrix element
    M_length = [length(char(M11)), length(char(M12)); 
	            length(char(M21)), length(char(M22))]

    
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   Velocity (N) Matrix    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    fprintf('\n\n-------------------------------------------------\n')
    fprintf('-------------- Velocity Matrix (N) --------------\n')
    fprintf('-------------------------------------------------\n')
    [N11, KN11] = coeffs(Equations(1), q1d);
    N11_qd_2 	= N11(1)  %for qd^2
    N11_qd   	= N11(2)  %for qd
    
    [N12, KN12] = coeffs(Equations(1), q2d);
    N12_qd_2 	= N12(1)  %for qd^2
    N12_qd   	= N12(2)  %for qd
    
    [N21, KN21] = coeffs(Equations(2), q1d);
    N21_qd_2 	= (N21(1))  %for qd^2
    N21_qd   	= (N21(1))  %for qd
    
    [N22, KN22] = coeffs(Equations(2), q2d);
    N22_qd_2 	= (N22(1))  %for qd^2
    N22_qd   	= (N22(1))  %for qd
	
	% Combine entire matrix
    N_qd_2 = [N11_qd_2,  N12_qd_2;  
	          N21_qd_2,  0]
    N_qd   = [N11_qd,    N12_qd;    
	          0,         0]
			  
	% Output text character length of each matrix element
    N_qd_2_length = [length(char(N11_qd_2)), length(char(N12_qd_2)); 
	                 length(char(N21_qd_2)), 0]
    N_qd_length   = [length(char(N11_qd)), length(char(N12_qd)); 
	                 0,                    0]


	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   Gravity (G) Matrix    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    fprintf('\n\n-------------------------------------------------\n')
    fprintf('-------------- Gravity Matrix (G)  --------------\n')
    fprintf('-------------------------------------------------\n')
    [G11, KG11] = coeffs(Equations(1), g);
    G11 		= (G11(1))*g
    
    [G21, KG21] = coeffs(Equations(2), g);
    G21 		= (G21(1))*g
	
	% Combine entire matrix
    G 		= [G11; 
	           G21]

	% Output text character length of each matrix element
    G_length = [length(char(G11)); 
	            length(char(G21))]
    
    
    
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%%%%%%   Saving Matrices to File    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	fprintf('Writing RIGHT stance matrix data to file ... ')
	% Open File
    fid = fopen('2Link_Equations_RIGHT.txt', 'w');
    
	% Write all Matrices to File
    fprintf(fid, 'M11 = %s;\n',  char(M11));
    fprintf(fid, 'M12 = %s;\n',  char(M12));
    fprintf(fid, 'M21 = M12;\n', char(M21));
    fprintf(fid, 'M22 = %s;\n',  char(M22));
    fprintf(fid, '\n\n');
    fprintf(fid, 'N11_qd_2 = %s;\n',  char(N11_qd_2));
    fprintf(fid, 'N12_qd_2 = %s;\n',  char(N12_qd_2));
    fprintf(fid, 'N21_qd_2 = %s;\n',  char(N21_qd_2));
    fprintf(fid, 'N22_qd_2 = 0;\n',   char(N22_qd_2));
    fprintf(fid, '\n\n');
    fprintf(fid, 'N11_qd = %s;\n',  char(N11_qd));
    fprintf(fid, 'N12_qd = %s;\n',  char(N12_qd));
    fprintf(fid, 'N21_qd = 0;\n',   char(N21_qd));
    fprintf(fid, 'N22_qd = 0;\n',   char(N22_qd));
    fprintf(fid, '\n\n');
    fprintf(fid, 'G11 = %s;\n',  char(G11));
    fprintf(fid, 'G21 = %s;\n',  char(G21));
    
	% Close File
    fclose(fid);
	fprintf('DONE\n\n')
end
