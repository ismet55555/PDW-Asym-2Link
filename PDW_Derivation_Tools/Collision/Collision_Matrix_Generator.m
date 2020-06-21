%=========================================================================================

% Passive Dynamic Walker (PDW) Simulation Model
% 2-Link (Compass Gait) Curved Foot Variation

%=========================================================================================
%
% Walker Heel Strike Collision Equation Generator/Tool
%
%=========================================================================================
%
% This script generates the heel strike matrices for the 2-link (compass gait) curved 
% foot passive dynamic walker.
%
% The script will only generate a selected side, left or right. That is, the resulting 
% collision matrices will either be for the left or right stance phase of the walker
%
% The resulting matrices are eventually used to  compute the walkers velocities after
% the walker has encountered a foot-to-ground heel strike.
%
% NOTES: 
% 		- "m" denotes pre-collision, "p" denotes post-collision ("minus" / "plus")
% 		- Coordinate System: [i, j, k]
%
% IMPORTANT:
%       - For final post-collision matrix, Qp, a column switch is needed.
%         That is:     Qp = [Qp11, Qp12;   --->   Qp = [Qp12, Qp11; 
%		                     Qp21, Qp22]                Qp22, Qp21]
%
%=========================================================================================
% Author: Ismet Handzic
% Date  : 07/02/2013
%=========================================================================================


clc			% Clear the command window
close all	% Close all open figures and plots
clear all   % Clear all variables in the workspace


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%    User Input for Left or Right Stance    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
user_input = input('Calculate LEFT or RIGHT Stance Dynamics? Enter "L" or "R"  : ', 's');


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%    LEFT Stance Walker Collision    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if (user_input == 'L') || (user_input == 'l')
    fprintf('\nComputing LEFT Stance/RIGHT Swing Heel Strike ... \n\n')
    
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   Define Symbolics    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    syms a1L b1L c1L  
    syms a1R b1R c1R 
    
    syms dm_a1L  dm_a1Lb1L  dm_LL
    syms dm_a1R  dm_a1Rb1R  dm_LR 
    
	syms mh
    syms ms1L mt1L 
    syms ms1R mt1R
    
    syms  q1dp q2dp q1
    syms  q1dm q2dm q2

    syms psi_LL psi_a1L psi_a1Lb1L 
    syms psi_LR psi_a1R psi_a1Rb1R
    
    syms psi psi2
    
    syms rL rR
	
	syms theta


    
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%                 Pivoting Distances                         %%%%%%%%%%%%
	%%%%%%%%%%%%%%   Conserved System 1: Whole walker, Rotating About Heel    %%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %mh - Hip Mass:
    B_mh_m = [-dm_LR*sin(psi_LR), ...
	           dm_LR*cos(psi_LR), 0];
    B_mh_p = [-dm_LR*sin(psi_LR), ...
	           dm_LR*cos(psi_LR), 0];

    %Left Leg Masses:
    %ms1L (lower shank)
    B_ms1L_m = [(c1L+b1L)*sin(q1) - dm_LR*sin(psi_LR), ...
	           -(c1L+b1L)*cos(q1) + dm_LR*cos(psi_LR), 0];
    B_ms1L_p = [(c1L+b1L)*sin(q1) - dm_LR*sin(psi_LR), ...
	           -(c1L+b1L)*cos(q1) + dm_LR*cos(psi_LR), 0];    
    %mt1L (upper thigh)
    B_mt1L_m = [c1L*sin(q1) - dm_LR*sin(psi_LR), ...
			   -c1L*cos(q1) + dm_LR*cos(psi_LR), 0];
    B_mt1L_p = [c1L*sin(q1) - dm_LR*sin(psi_LR), ...
			   -c1L*cos(q1) + dm_LR*cos(psi_LR), 0];

    %Right Leg Masses:
    %ms1R (lower shank)
    B_ms1R_m = [-dm_a1R*sin(psi_a1R), ...
	             dm_a1R*cos(psi_a1R), 0];
    B_ms1R_p = [-dm_a1R*sin(psi_a1R), ...
	             dm_a1R*cos(psi_a1R), 0];
    %mt1R (upper thigh)
    B_mt1R_m = [-dm_a1Rb1R*sin(psi_a1Rb1R), ...
				 dm_a1Rb1R*cos(psi_a1Rb1R), 0];
    B_mt1R_p = [-dm_a1Rb1R*sin(psi_a1Rb1R), ...
				 dm_a1Rb1R*cos(psi_a1Rb1R), 0];

	
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%                 Pivoting Distances                    %%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%   Conserved System 2: Swing Leg, Rotating About Hip   %%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %mt1L (upper thigh w.r.t. hip)
    H_mt1L_m = [c1L*sin(q1), ...
	           -c1L*cos(q1), 0];
    H_mt1L_p = [c1L*sin(q1), ...
	           -c1L*cos(q1), 0];
    %ms1L (lower shank w.r.t. hip)
    H_ms1L_m = [(c1L+b1L)*sin(q1), ...
	           -(c1L+b1L)*cos(q1), 0];
    H_ms1L_p = [(c1L+b1L)*sin(q1), ...
	           -(c1L+b1L)*cos(q1), 0];

    
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   Mass Velocities    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %Hip Mass:
    v_mh_m = [-dm_LL*q1dm*cos(psi_LL), ...
	          -dm_LL*q1dm*sin(psi_LL) , 0];   %w.r.t Left Heel
    v_mh_p = [-dm_LR*q2dp*cos(psi_LR), ...
	          -dm_LR*q2dp*sin(psi_LR) , 0];   %w.r.t Right Heel (B)

    %Left Leg Masses:
    %ms1L (lower shank)
    v_ms1L_m = [-dm_a1L*q1dm*cos(psi_a1L), ...
	            -dm_a1L*q1dm*sin(psi_a1L) , 0];
    v_ms1L_p = [(c1L+b1L)*q1dp*cos(q1) - dm_LR*q2dp*cos(psi_LR), ...
				(c1L+b1L)*q1dp*sin(q1) - dm_LR*q2dp*sin(psi_LR), 0];
    %mt1L (upper thigh)
    v_mt1L_m = [-dm_a1Lb1L*q1dm*cos(psi_a1Lb1L ), ...
	            -dm_a1Lb1L*q1dm*sin(psi_a1Lb1L ) , 0];
    v_mt1L_p = [(c1L)*q1dp*cos(q1) - dm_LR*q2dp*cos(psi_LR), ...
				(c1L)*q1dp*sin(q1) - dm_LR*q2dp*sin(psi_LR), 0];

    %Right Leg Masses:
    %ms1R (lower shank)
    v_ms1R_m = [(c1R+b1R)*q2dm*cos(q2) - dm_LL*q1dm*cos(psi_LL), ...
	            (c1R+b1R)*q2dm*sin(q2) - dm_LL*q1dm*sin(psi_LL), 0];
    v_ms1R_p = [-dm_a1R*q2dp*cos(psi_a1R), ...
	            -dm_a1R*q2dp*sin(psi_a1R), 0];
    %mt1R (upper thigh)
    v_mt1R_m = [(c1R)*q2dm*cos(q2) - dm_LL*q1dm*cos(psi_LL), ...
	            (c1R)*q2dm*sin(q2) - dm_LL*q1dm*sin(psi_LL), 0];
    v_mt1R_p = [-dm_a1Rb1R*q2dp*cos(psi_a1Rb1R), ...
	            -dm_a1Rb1R*q2dp*sin(psi_a1Rb1R) , 0];

    
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%           Conservation of Angular Momentum                 %%%%%%%%%%%%
	%%%%%%%%%%%%%%   Conserved System 1: Whole walker, Rotating About Heel    %%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %Pre-Collision:
    Eq1m = cross(mh   * B_mh_m,   v_mh_m  ) ... %-- hip
         + cross(ms1L * B_ms1L_m, v_ms1L_m) ... %\left leg
         + cross(mt1L * B_mt1L_m, v_mt1L_m) ... %/
         + cross(ms1R * B_ms1R_m, v_ms1R_m) ... %\right leg
         + cross(mt1R * B_mt1R_m, v_mt1R_m);    %/
    Eq1m = collect(Eq1m(3));

    %Post-Collision
    Eq1p = cross(mh*B_mh_p,       v_mh_p) ...   %-- hip
         + cross(ms1L * B_ms1L_p, v_ms1L_p) ... %\left leg
         + cross(mt1L * B_mt1L_p, v_mt1L_p) ... %/
         + cross(ms1R * B_ms1R_p, v_ms1R_p) ... %\right leg
         + cross(mt1R * B_mt1R_p, v_mt1R_p);    %/  
    Eq1p = collect(Eq1p(3));

    
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%           Conservation of Angular Momentum              %%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%     Conserved System 2: Swing Leg, Rotating About Hip   %%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %Pre-Collision:
    Eq2m = cross(mt1L * H_mt1L_m, v_mt1L_m) ...
         + cross(ms1L * H_ms1L_m, v_ms1L_m);
    Eq2m = collect(Eq2m(3));

    %Post-Collision
    Eq2p = cross(mt1L * H_mt1L_p, v_mt1L_p) ...
         + cross(ms1L * H_ms1L_p, v_ms1L_p);
    Eq2p = collect(Eq2p(3));

    
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%%%%%%   PRE-Collision (Qm) Matrix    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    fprintf('\n\n-------------------------------------------------\n')
    fprintf('---------- PRE-Collision Matrix (Qm) ------------\n')
    fprintf('-------------------------------------------------\n')
    [Qm11, TQm11] = coeffs(Eq1m, q1dm);
    Qm11 = simple(Qm11(1))

    [Qm12, TQm12] = coeffs(Eq1m, q2dm);
    Qm12 = simple(Qm12(1))

    [Qm21, TQm21] = coeffs(Eq2m, q1dm);
    Qm21 = simple(Qm21(1))
	
    [Qm22, TQm22] = coeffs(Eq2m, q2dm);
    Qm22 = 0

	% Assemble pre-collision matrix
    Qm = [Qm11, Qm12; 
		  Qm21, Qm22];
		  
	% Output the character length for each matrix element
    Qm_length = [length(char(Qm11)), length(char(Qm12)); ... 
                 length(char(Qm21)), length(char(Qm22))]


	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%%%%%%   POST-Collision (Qp) Matrix    %%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    fprintf('\n\n-------------------------------------------------\n')
    fprintf('--------- POST-Collision Matrix (Qp) ------------\n')
    fprintf('-------------------------------------------------\n')
    [Qp11, TQp11] = coeffs(Eq1p, q1dp);
    Qp11 = simple(Qp11(1))

    [Qp12, TQp12] = coeffs(Eq1p, q2dp);
    Qp12 = simple(Qp12(1))

    [Qp21, TQp21] = coeffs(Eq2p, q1dp);
    Qp21 = simple(Qp21(1))

    [Qp22, TQp22] = coeffs(Eq2p, q2dp);
    Qp22 = simple(Qp22(1))
	
	% Assemble post-collision matrix
	% IMPORTANT: 
	%        - Note that switch in this post-collision matrix columns is needed.
	%          However not sure why ??
    Qp = [Qp12, Qp11; 
		  Qp22, Qp21]
		  
	% Output the character length for each matrix element
    Qp_length = [length(char(Qp12)), length(char(Qp11));...
                 length(char(Qp22)), length(char(Qp21))]

    

	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%%%%%%   Saving Matrices to File    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	fprintf('Writing LEFT stance collision matrix data to file ... ')
	
	% Open File
    output_file = fopen('2L_Heel_Strike_Collision_Equations_LEFT.txt', 'w');

	% Write all Matrices to File
    fprintf(output_file, 'Qm11 = %s;\n',  char(Qm11));
    fprintf(output_file, 'Qm12 = %s;\n',  char(Qm12));
    fprintf(output_file, 'Qm21 = %s;\n',  char(Qm21));
    fprintf(output_file, 'Qm22 = %s;\n',  char(Qm22));
    fprintf(output_file, '\n\n');
    fprintf(output_file, 'Qp11 = %s;\n',  char(Qp12));
    fprintf(output_file, 'Qp12 = %s;\n',  char(Qp11)); %notice the switch
    fprintf(output_file, 'Qp21 = %s;\n',  char(Qp22));
    fprintf(output_file, 'Qp22 = %s;\n',  char(Qp21));

	% Close File
    fclose(output_file);
	fprintf('DONE\n\n')
end




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%    RIGHT Stance Walker Collision    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if (user_input == 'R') || (user_input == 'r')
    fprintf('\nComputing RIGHT Stance/LEFT Swing Heel Strike ... \n\n')
	
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   Define Symbolics    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    syms a1L b1L c1L  
    syms a1R b1R c1R 
    
    syms dm_a1L  dm_a1Lb1L  dm_LL
    syms dm_a1R  dm_a1Rb1R  dm_LR 
    
	syms mh
    syms ms1L mt1L 
    syms ms1R mt1R
    
    syms  q1dp q2dp q1
    syms  q1dm q2dm q2

    syms psi_LL psi_a1L psi_a1Lb1L 
    syms psi_LR psi_a1R psi_a1Rb1R
    
    syms psi psi2
    
    syms rL rR
	
	syms theta


	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%                 Pivoting Distances                         %%%%%%%%%%%%
	%%%%%%%%%%%%%%   Conserved System 1: Whole walker, Rotating About Heel    %%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %mh - Hip Mass:
    B_mh_m = [-dm_LL*sin(psi_LL), ...
	dm_LL*cos(psi_LL), 0];
    B_mh_p = [-dm_LL*sin(psi_LL), ...
	dm_LL*cos(psi_LL), 0];

    %Right Leg Masses:
    %ms1R (lower shank)
    B_ms1R_m = [(c1R+b1R)*sin(q1) - dm_LL*sin(psi_LL), ...
			   -(c1R+b1R)*cos(q1) + dm_LL*cos(psi_LL), 0];
    B_ms1R_p = [(c1R+b1R)*sin(q1) - dm_LL*sin(psi_LL), ...
			   -(c1R+b1R)*cos(q1) + dm_LL*cos(psi_LL), 0];    
    %mt1R (upper thigh)
    B_mt1R_m = [c1R*sin(q1) - dm_LL*sin(psi_LL), ...
			   -c1R*cos(q1) + dm_LL*cos(psi_LL), 0];
    B_mt1R_p = [c1R*sin(q1) - dm_LL*sin(psi_LL), ...
			   -c1R*cos(q1) + dm_LL*cos(psi_LL), 0];

    %Left Leg Masses:
    %ms1L (lower shank)
    B_ms1L_m = [-dm_a1L*sin(psi_a1L), ...
		         dm_a1L*cos(psi_a1L), 0];
    B_ms1L_p = [-dm_a1L*sin(psi_a1L), ...
			     dm_a1L*cos(psi_a1L), 0];
    %mt1L (upper thigh)
    B_mt1L_m = [-dm_a1Lb1L*sin(psi_a1Lb1L), ...
				 dm_a1Lb1L*cos(psi_a1Lb1L), 0];
    B_mt1L_p = [-dm_a1Lb1L*sin(psi_a1Lb1L), ...
	             dm_a1Lb1L*cos(psi_a1Lb1L), 0];

    
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%                 Pivoting Distances                    %%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%   Conserved System 2: Swing Leg, Rotating About Hip   %%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %mt1R (upper thigh w.r.t. hip)
    H_mt1R_m = [c1R*sin(q1), ...
	           -c1R*cos(q1), 0];
    H_mt1R_p = [c1R*sin(q1), ...
	           -c1R*cos(q1), 0];
    %ms1R (lower shank w.r.t. hip)
    H_ms1R_m = [(c1R+b1R)*sin(q1), ...
	           -(c1R+b1R)*cos(q1), 0];
    H_ms1R_p = [(c1R+b1R)*sin(q1), ...
	           -(c1R+b1R)*cos(q1), 0];

    
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   Mass Velocities    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
    %Hip Mass:
    v_mh_m = [-dm_LR*q1dm*cos(psi_LR), ...
	          -dm_LR*q1dm*sin(psi_LR) , 0];   %w.r.t Right Heel
    v_mh_p = [-dm_LL*q2dp*cos(psi_LL), ...
	          -dm_LL*q2dp*sin(psi_LL) , 0];   %w.r.t Left Heel (B)

    %Right Leg Masses:
    %ms1R (lower shank)
    v_ms1R_m = [-dm_a1R*q1dm*cos(psi_a1R), ...
	            -dm_a1R*q1dm*sin(psi_a1R) , 0];
    v_ms1R_p = [(c1R+b1R)*q1dp*cos(q1) - dm_LL*q2dp*cos(psi_LL), ...
	            (c1R+b1R)*q1dp*sin(q1) - dm_LL*q2dp*sin(psi_LL), 0];
    %mt1R (upper thigh)
    v_mt1R_m = [-dm_a1Rb1R*q1dm*cos(psi_a1Rb1R ), ...
	            -dm_a1Rb1R*q1dm*sin(psi_a1Rb1R ) , 0];
    v_mt1R_p = [(c1R)*q1dp*cos(q1) - dm_LL*q2dp*cos(psi_LL), ...
	            (c1R)*q1dp*sin(q1) - dm_LL*q2dp*sin(psi_LL), 0];

    %Left Leg Masses:
    %ms1L (lower shank)
    v_ms1L_m = [(c1L+b1L)*q2dm*cos(q2) - dm_LR*q1dm*cos(psi_LR), ...
	            (c1L+b1L)*q2dm*sin(q2) - dm_LR*q1dm*sin(psi_LR), 0];
    v_ms1L_p = [-dm_a1L*q2dp*cos(psi_a1L), ...
	            -dm_a1L*q2dp*sin(psi_a1L) , 0];
    %mt1L (upper thigh)
    v_mt1L_m = [(c1L)*q2dm*cos(q2) - dm_LR*q1dm*cos(psi_LR), ...
	            (c1L)*q2dm*sin(q2) - dm_LR*q1dm*sin(psi_LR), 0];
    v_mt1L_p = [-dm_a1Lb1L*q2dp*cos(psi_a1Lb1L), ...
	            -dm_a1Lb1L*q2dp*sin(psi_a1Lb1L) , 0];


	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%           Conservation of Angular Momentum                 %%%%%%%%%%%%
	%%%%%%%%%%%%%%   Conserved System 1: Whole walker, Rotating About Heel    %%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %Pre-Collision:
    Eq1m = cross(mh   * B_mh_m,   v_mh_m  ) ... %-- hip
         + cross(ms1R * B_ms1R_m, v_ms1R_m) ... %\right leg
         + cross(mt1R * B_mt1R_m, v_mt1R_m) ... %/
         + cross(ms1L * B_ms1L_m, v_ms1L_m) ... %\left leg
         + cross(mt1L * B_mt1L_m, v_mt1L_m);    %/
    Eq1m = collect(Eq1m(3));

    %Post-Collision
    Eq1p = cross(mh   * B_mh_p,   v_mh_p  ) ... %-- hip
         + cross(ms1R * B_ms1R_p, v_ms1R_p) ... %\right leg
         + cross(mt1R * B_mt1R_p, v_mt1R_p) ... %/
         + cross(ms1L * B_ms1L_p, v_ms1L_p) ... %\left leg
         + cross(mt1L * B_mt1L_p, v_mt1L_p);    %/  
    Eq1p = collect(Eq1p(3));


	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%           Conservation of Angular Momentum              %%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%     Conserved System 2: Swing Leg, Rotating About Hip   %%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %Pre-Collision:
    Eq2m = cross(mt1R * H_mt1R_m, v_mt1R_m) ...
         + cross(ms1R * H_ms1R_m, v_ms1R_m);
    Eq2m = collect(Eq2m(3));

    %Post-Collision
    Eq2p = cross(mt1R * H_mt1R_p, v_mt1R_p) ...
         + cross(ms1R * H_ms1R_p, v_ms1R_p);
    Eq2p = collect(Eq2p(3));


	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%%%%%%   PRE-Collision (Qm) Matrix    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    fprintf('\n\n-------------------------------------------------\n')
    fprintf('---------- PRE-Collision Matrix (Qm) ------------\n')
    fprintf('-------------------------------------------------\n')
    [Qm11, T] = coeffs(Eq1m, q1dm);
    Qm11 = simple(Qm11(1))

    [Qm12, T] = coeffs(Eq1m, q2dm);
    Qm12 = simple(Qm12(1))

    [Qm21, T] = coeffs(Eq2m, q1dm);
    Qm21 	  = simple(Qm21(1))

    [Qm22, T] = coeffs(Eq2m, q2dm);
    Qm22      = 0

	% Assemble pre-collision matrix
    Qm = [Qm11, Qm12; 
		  Qm21, Qm22];

	% Output the character length for each matrix element
    Qm_length = [length(char(Qm11)), length(char(Qm12)); ... 
                 length(char(Qm21)), length(char(Qm22))]

             
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%%%%%%   POST-Collision (Qp) Matrix    %%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    fprintf('\n\n-------------------------------------------------\n')
    fprintf('--------- POST-Collision Matrix (Qp) ------------\n')
    fprintf('-------------------------------------------------\n')
    [Qp11, T] = coeffs(Eq1p, q1dp);
    Qp11 = simple(Qp11(1))

    [Qp12, T] = coeffs(Eq1p, q2dp);
    Qp12 = simple(Qp12(1))

    [Qp21, T] = coeffs(Eq2p, q1dp);
    Qp21 = simple(Qp21(1))

    [Qp22, T] = coeffs(Eq2p, q2dp);
    Qp22 = simple(Qp22(1))
	
	% Assemble post-collision matrix
	% IMPORTANT: 
	%        - Note that switch in this post-collision matrix columns is needed.
	%          However not sure why ??
    Qp = [Qp12, Qp11;
	      Qp22, Qp21]  %Switch columns is needed, however not sure why ??
		  
	% Output the character length for each matrix element
    Qp_length = [length(char(Qp12)), length(char(Qp11)); ...
                 length(char(Qp22)), length(char(Qp21))]

             
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%%%%%%   Saving Matrices to File    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	fprintf('Writing RIGHT stance collision matrix data to file ... ')
	
	% Open File
    output_file = fopen('2L_Heel_Strike_Collision_Equations_RIGHT.txt', 'w');
    
    fprintf(output_file, 'Qm11 = %s;\n',  char(Qm11));
    fprintf(output_file, 'Qm12 = %s;\n',  char(Qm12)); 
    fprintf(output_file, 'Qm21 = %s;\n',  char(Qm21));
    fprintf(output_file, 'Qm22 = %s;\n',  char(Qm22));
    fprintf(output_file, '\n\n');
    fprintf(output_file, 'Qp11 = %s;\n',  char(Qp12));
    fprintf(output_file, 'Qp12 = %s;\n',  char(Qp11)); %notice the switch
    fprintf(output_file, 'Qp21 = %s;\n',  char(Qp22));
    fprintf(output_file, 'Qp22 = %s;\n',  char(Qp21));
    
	% Close File
    fclose(output_file);
	fprintf('DONE\n\n')      
end
