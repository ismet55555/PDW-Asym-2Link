%3-Link PDW Model with Curved Feet
%Ismet Handzic
%01/15/2014
%REEDLab - University of SouthFlorida


close all
clc
clear all
    
fprintf('\n=======================================')
fprintf('\n======== SIMULATION  STARTED ==========')
fprintf('\n=======================================\n')

    
%======================== Simulation Control ======================
% Time Step (s)
dt = 0.0015;
 
% Total Steps Taken
total_steps = 4;

% Simulation Animation
display = 1; 
% 
% Gravity (m/s^2)
g = 9.81;   



%========================== Ramp Angle =============================
theta = asin(08.55/144);
RampAngle = theta*(180/pi);
fprintf('Ramp Angle: %2.3f Degrees\n', RampAngle);
fprintf('            %2.3f Radians\n\n',   theta);


%==================== PDW Masses and Lengths ========================
%   Masses
mh =   2.000000;   %Hip Mass (torso)

ms1L = 0.050000;   %lower shank left 
ms2L = 0.150000;   %upper shank left 
mt2L = 0.800000;   %lower thigh left 
mt1L = 1.000000;   %upper thigh left  

ms1R = 0.050000;   %lower shank right 
ms2R = 0.150000;   %upper shank right
mt2R = 0.800000;   %lower thigh right
mt1R = 1.000000;   %upper thigh right



mh = 4.0;

ms1L = 0.11;   %low shank left
ms2L = 0.7;     %up shank left
mt1L = 2.1;   %up thigh left
mt2L = 0.1;     %low thigh left

ms1R = 0.11;  %low shank right 
ms2R = 0.7;     %up shank right
mt1R = 2.1;   %up thigh right
mt2R = 0.1;     %low thigh right



% Walker Mass Multipiers for Display
mhmul = 100;      mtmul = 100;      msmul = 100;

mhmul = 30;
mtmul = 30;
msmul = 40;

%   Mass Distribution Distances
%Left Leg
a1L = 1/5;              %left shank
b1L = 1/5;   
c1L = 1/5;
lsL = a1L + b1L + c1L;
c2L = 1/5;              %left thigh
b2L = 1/5;   
a2L = 1/5;
ltL = c2L + b2L + a2L;
LL = lsL + ltL;

%Right Leg
a1R = 1/5;              %right shank
b1R = 1/5;   
c1R = 1/5;
lsR = a1R + b1R + c1R;
c2R = 1/5;              %right thigh
b2R = 1/5;   
a2R = 1/5;
ltR = c2R + b2R + a2R;
LR = lsR + ltR;
  




%   Left Leg
a1L = 0.001;
b1L = 0.28;
c1L = 0.22;
a2L = 0.22;
b2L = 0.005;
c2L = 0.28;
lsL = a1L+b1L+c1L;  
ltL = a2L+b2L+c2L;
LL = lsL+ltL;

%   Right Leg
a1R = 0.001;% + 0.15;
b1R = 0.28;
c1R = 0.22;
a2R = 0.22;
b2R = 0.005;
c2R = 0.28;
lsR = a1R+b1R+c1R;
ltR = a2R+b2R+c2R;
LR = lsR+ltR;









%===================== Foot Shape Parameters ========================
%    Form:   r = rLa + rLb*angle |   + rLc*sin(angle + rLd)
rLa = 0.4;    rLb = -0.0200;   dL = 0.1;
rRa = 0.4;    rRb = -0.0200;   dR = 0.1;

rLb = 0;
rRb = 0;


fprintf('Left  Foot Radius: %s + %s*Angle\n', num2str(rLa), num2str(rLb));
fprintf('Left  Foot Offset: %s\n\n', num2str(dL));
fprintf('Right Foot Radius: %s + %s*Angle\n', num2str(rRa), num2str(rRb));
fprintf('Right Foot Offset: %s\n\n', num2str(dR));


%======================= Intial Conditions ==========================
% %Backward Foot
q =  [0.1787;    -0.3804;   -0.8804];     
qd = [-1.2276;    1.5313;   1.5313];   

%Forward Foot
q =  [0.1340;    -0.4804;   -0.9804]; 
qd = [-1.300;     1.1485;   1.1485];

q =  [0.2444; -0.3593; -0.3693];
qd = [-1.1363; 0.3481; 0.3481];



%========================= Simulation Output  =======================
energy          = 1;        %Energy Plot
   energy_cycle = 0;        %  Energy cycle plot
force           = 1;        %Kinetic Plot
step_length     = 1;        %Step Length Plot
step_time       = 0;        %Step Time Plot
limit_cycle     = 0;        %Limit Cycle Plot
angle           = 0;        %Leg and Hip Angle Plot
angular_vel     = 0;        %Leg and Hip Angular Velocity Plot


save_walking_figure = 0;    %Save Walking Figure to make .avi video file
print_count = 0;            %picture counter for saved animation
velocity_disp = 0;


time_plot = 1;

% Data Collection
changed_variable = ms1L;
params.data_name = 'DATA';  %string to use to text file name
params.other = 'other';     %incase you want to make another string for something (unused)

data_name = params.data_name;
masscount = 0;
param_count = 1;
data_name = params.data_name;

%Start Simulation Timer
tic


% %Searching and opening up multiple processors for computing
% matlabpool

Step_Length_Asym_Percent = [];
Step_Time_Asym_Percent = [];
Symmetry_Fx = [];
Symmetry_Fy = [];
Offset = [];

%VAR_loop = 0.0 : 0.025 : 0.450;
%q1_loop = 0.1260 : 0.027 : 0.2340;
%parfor m = 1 : length(VAR_loop) 
%     
%     %q2
%     for o = -0.2800 : -0.060 : -0.5200
%     
%     %q1d
%     for n = -0.9100 : -0.195 : -1.6900
%         
%     %q2d
%     for p = 0.8750 : 0.1875 : 1.6250
%         
%     %theta  
%     for k = 0.0350 : 0.015 : 0.0650 
%         theta = k;
%         
%     for rLb = -0.19 : 0.0316666667 : 0.19
%         rRb = rLb;
%         
%         
%     q =     [q1_loop(m);    o;   0];     
%     qd =    [n;             p;   0];      
%   

m = 1;   
VAR_loop(m) = 0.0;   %0 and 0.425 - damping: 3LL Knee - 0.275
%a1R = 0.005 + VAR_loop(m);
%rLa = 0.2 + VAR_loop(m);
%rRa = 0.2;

% lsL = a1L+b1L+c1L;  
% ltL = a2L+b2L+c2L;
% LL = lsL+ltL;
% LL2 = lsL+ltL+rLa;
% 
% lsR = a1R+b1R+c1R;
% ltR = a2R+b2R+c2R;
% LR = lsR+ltR;
% LR2 = lsR+ltR+rRa;

        changed_variable = 0;
        %--------------------------------------------------- 
        %Loop this part

        FootData = [rLa, rLb, dL;   rRa, rRb, dR];

        %Preping/consolidating parameters to send into functions
        parameters = [mt1R, mt2R, ms2R, ms1R, a1R, b1R, c1R, c2R, b2R, a2R, lsR, ltR, LR, ...
                      mt1L, mt2L, ms2L, ms1L, a1L, b1L, c1L, c2L, b2L, a2L, lsL, ltL, LL, mh, g, dt, theta, ...
                      time_plot, total_steps, display, energy, step_length, limit_cycle,...
                      save_walking_figure, print_count, mhmul, mtmul, msmul, ...
                      changed_variable, velocity_disp, force, energy_cycle, angle, angular_vel, step_time];

                  

        %Calling Walker_Control function to start simulation
        [stop_condition, steps, successful, right_step_length, left_step_length, right_step_time, left_step_time, EnergyStat, ForcesStat]...
            = Walker_Control(q, qd, parameters, params, total_steps, FootData);

        
            
        begin = round(0.20*length(left_step_length)); %start from 20% of data
       
        %SAVING PARAMETERS TO TEXT FILE
        %all_params = [q(1,1), q(2,1), qd(1,1) ,qd(2,1), theta, rLa, rLb, dL, rRa, rRb, dR ...
                       %mean(EnergyStat(13:2:end,1)), mean(EnergyStat(14:2:end,1)), mean(EnergyStat(13:1:end,1)), ...
                       %mean(ForcesStat(13:2:end,1)), mean(ForcesStat(13:2:end,2)), mean(ForcesStat(14:2:end,1)), mean(ForcesStat(14:2:end,2)), ...
                       %mean(left_step_length(begin:end)), mean(right_step_length(begin:end)), steps, successful];

        %fid = fopen(strcat([data_name '.txt']),'a');  %Open text file to save model data
        
        %Write all parameter to text file
        %fprintf(fid,'%6.4f\t%6.4f\t%6.4f\t%6.4f\t%6.4f\t%6.4f\t%6.4f\t%6.4f\t%6.4f\t%6.4f\t%6.4f\t%6.4f\t%6.4f\t%6.4f\t%6.4f\t%6.4f\t%6.4f\t%6.4f\t%6.4f\t%6.4f\t%6.4f\t%6.4f\t\r\n', all_params);
        
        %fclose(fid);   %Close text file

        %-----------------------------------------------------
        
        if (successful == 1)
           fprintf('\nDid not fall.   :) \n') 
           
           Asym_Percent = 100*abs((mean(left_step_length(begin:end))/mean(right_step_length(begin:end)))-1);
           if (  abs((mean(left_step_length(begin:end))/mean(right_step_length(begin:end)))-1) < 0.04 )
               fprintf('Spacial Symmetry!  Asymmetry: %g%%\n', Asym_Percent)             
           end
           Step_Length_Asym_Percent(m) = Asym_Percent;
           
           Asym_Percent = 100*abs((mean(left_step_time(begin:end))/mean(right_step_time(begin:end)))-1);
           if (  abs((mean(left_step_time(begin:end))/mean(right_step_time(begin:end)))-1) < 0.04 )
               fprintf('Temporal Symmetry!  Asymmetry: %g%%\n', Asym_Percent)             
           end
           Step_Time_Asym_Percent(m) = Asym_Percent;
           
           fprintf('Offset: %g m\n', VAR_loop(m))
           Offset(m) = VAR_loop(m);
           
           
            Symmetry_Fx(m) = abs(( mean(ForcesStat(begin:2:end,1))/mean(ForcesStat(begin+1:2:end,1)) ) - 1) * 100;
            Symmetry_Fy(m) = abs(( mean(ForcesStat(begin:2:end,2))/mean(ForcesStat(begin+1:2:end,2)) ) - 1) * 100;
           
           
        else
           fprintf('\nPDW Fell.  :( \n') 
        end
        
%     end   
%     end
%     end
%     end 
%     end
%   

fprintf('\n\n==================================\n\n') 
%end

% figure(1000);
% plot(Offset, Step_Length_Asym_Percent, 'ob', 'markersize', 8,'MarkerFaceColor','b')
% hold on
% plot(Offset, Step_Time_Asym_Percent, 'sr', 'markersize', 8, 'MarkerFaceColor','r')
% 
% plot(Offset, Symmetry_Fx, 'vb', 'markersize', 9,'MarkerFaceColor','b')
% plot(Offset, Symmetry_Fy, 'pr', 'markersize', 9,'MarkerFaceColor','r')
% 
% set(get(gcf,'CurrentAxes'),'FontName','Times New Roman','FontSize',15);
% xlabel('Foot Curve Asymmetry (m)')
% ylabel('Percent Asymmetry (%)')
% %title('Step Time','FontName','Times New Roman','FontSize',20)
% legend('Step Length','Step Time','X-GRF','Y-GRF')
% set(gcf, 'PaperPosition', [1 1 4.2 4.0]);
% set(gcf,'position', [768   575   743   420])
% axis auto
% grid on


fprintf('\n=======================================')
fprintf('\n======== SIMULATION COMPLETE ==========')
fprintf('\n=======================================\n')
fprintf('    Total Simulation Runtime: %3.2fs\n',toc)

%closing multiple processors
%matlabpool close