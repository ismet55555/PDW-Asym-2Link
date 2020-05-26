%TWO LINK MECHANISM
%LEFT

function [q, qdp, stop_condition] = two_link_left_masses(q,qd,parameters,params)

%Define all parameters by assigning them indecies of the incoming
%"parameters" 
mh = parameters(1);
mt1R = parameters(2);
mt2R = parameters(3);
ms1R = parameters(4);
ms2R = parameters(5);
a1R = parameters(6);
b1R = parameters(7);
c1R = parameters(8);
a2R = parameters(9);
b2R = parameters(10);
c2R = parameters(11);
lsR = parameters(12);
ltR = parameters(13);
LR = parameters(14);
mt1L = parameters(15);
mt2L = parameters(16);
ms1L = parameters(17);
ms2L = parameters(18);
a1L = parameters(19);
b1L = parameters(20);
c1L = parameters(21);
a2L = parameters(22);
b2L = parameters(23);
c2L = parameters(24);
lsL = parameters(25);
ltL = parameters(26);
LL = parameters(27);
g = parameters(28);
dt = parameters(29);
theta = parameters(30);
time_plot = parameters(31);
total_steps = parameters(32);
display = parameters(33);
energy = parameters(34);
step_length = parameters(35);
cycle = parameters(36);
save_walking_figure = parameters(37);
print_count = parameters(38);
mhmul = parameters(39);
mtmul = parameters(40);
msmul = parameters(41);
changed_variable = parameters(42);
velocity_disp= parameters(43);

%Strings
data_name = params.data_name;
other = params.other;

stop_condition = 0;
count = 2;

%starting time
time(1) = 0;  


x_heel = -sin(q(1,end))*LL + sin(q(2,end))*LR;
y_heel = abs(cos(q(1,end))*LL) - abs(cos(q(2,end))*LR);
ramp = -(tan(theta)*x_heel);

ramp = (tan(theta)*x_heel);



while(y_heel >= ramp || (qd(2,count-1) > 0))  
    x_heel = -sin(q(1,count-1))*LL + sin(q(2,count-1))*LR;
    y_heel = abs(cos(q(1,count-1))*LL) - abs(cos(q(2,count-1))*LR);
	ramp = -(tan(theta)*x_heel);  
    
    if (qd(2,count-1) > 0)     
        ramp = ramp - 5;  
    end

	
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    DYNAMICS    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    q1 = q(1,count-1);
	q2 = q(2,count-1);
    
    M11 = mt2L*(a1L + b1L + c1L + c2L)^2 + mt1L*(a1L + b1L + b2L + c1L + c2L)^2 + LL^2*mh + LL^2*mt1R + LL^2*mt2R;
    M12 = -LL*cos(q1 - q2)*(a2R*mt1R + a2R*mt2R + b2R*mt2R);
    M21 = -LL*cos(q1 - q2)*(a2R*mt1R + a2R*mt2R + b2R*mt2R);
    M22 = mt2R*(a2R + b2R)^2 + a2R^2*mt1R;
    
    N12 = -LL*sin(q1 - q2)*(a2R*mt1R + a2R*mt2R + b2R*mt2R);
    N21 =  LL*sin(q1 - q2)*(a2R*mt1R + a2R*mt2R + b2R*mt2R);
    
    G11 = -g*(mt2L*sin(q1)*(a1L + b1L + c1L + c2L) + mt1L*sin(q1)*(a1L + b1L + b2L + c1L + c2L) + LL*mh*sin(q1) + LL*mt1R*sin(q1) + LL*mt2R*sin(q1));
    G21 = g*sin(q2)*(a2R*mt1R + a2R*mt2R + b2R*mt2R);
    
    H = [M11, M12; M21, M22];
    B = [0,   N12; N21, 0];
    G = [G11; G21];

	% Calculating angular accelerations
    qdd2 = inv(H)*(-B*qd(1:2,count-1).^2-G);
   
    %Numeric integration to find angular velocities (qd) and positions (q)
    qdd(:,count) = inv(H)*(-B*qd(1:2,count-1).^2-G);
    qdd(:,count) = qdd2;
    qd(:,count)  = qd(:,count-1) + qdd(:,count)*dt;
    q(:,count)   = q(:,count-1) + qd(:,count)*dt;
    
    % Calculating the energy for the walker
    E_cont(count) = energy_right_masses([q(1,end);q(2,end);qd(2,end)],[qd(1,end);qd(2,end);qd(2,end)],parameters,params);
    
	% Time step for numeric integration
    time(count) = time(count-1) + dt;
    
	% Add to number of iterations for this walking phase
    count = count + 1;
    
    %if the loop exceeds some iteration - stop
    if(count > 150000)
        stop_condition = 1;
        break;
    end
end

%limit cycle plot
if (cycle)
    figure(127)
    plot(q(2,:),qd(2,:),'g-')
    hold on
    plot(q(2,1),qd(2,1),'*m')
end

% Stopping condition check to make sure it ran more then two iterations
if(length(q) < 4)
    disp('WARNING: two_link_model_left_PDW ran less than two iterations')
    stop_condition = 1;
elseif(display)
    
    figure(999);
	% TODO: Add a figure handle somehow for display!

	
    
    %Plotting of simulation
    for loop = 1:100:length(q)
		
		% Set up the plot
		plot(NaN, NaN)
		hold on
		axis([-.5 .5 -.05 1.25])
		grid on
		
		% Find the height of the walker hip
        height_adj = LL*cos(q(1,loop));
        
		% Plot Legs
        plot([0 LL*sin(q(1,loop))],height_adj+[0 -LL*cos(q(1,loop))],'-r','LineWidth',2)
        plot([0 LR*sin(q(2,loop))],height_adj+[0 -LR*cos(q(2,loop))],'-b','LineWidth',2)
		
        
        plot(a2R*sin(q(2,loop)),height_adj-a2R*cos(q(2,loop)),'.k','MarkerSize',max([mt1R*mtmul 1]))
        plot((a2R+b2R)*sin(q(2,loop)),height_adj-(a2R+b2R)*cos(q(2,loop)),'.k','MarkerSize',max([mt2R*mtmul 1]))
		
        plot(0,height_adj,'.k','MarkerSize',max([mh*mhmul 1]))
		
        plot(ltR*sin(q(2,loop))+c1R*sin(q(2,loop)),height_adj-ltR*cos(q(2,loop))-c1R*cos(q(2,loop)),'.k','MarkerSize',max([ms2R*msmul 1]))
        plot(ltR*sin(q(2,loop))+(c1R+b1R)*sin(q(2,loop)),height_adj-ltR*cos(q(2,loop))-(c1R+b1R)*cos(q(2,loop)),'.k','MarkerSize',max([ms1R*msmul 1]))
        plot((ltL+c1L+b1L)*sin(q(1,loop)),height_adj-(ltL+c1L+b1L)*cos(q(1,loop)),'.k','MarkerSize',max([ms1L*msmul 1]))
        plot((ltL+c1L)*sin(q(1,loop)),height_adj-(ltL+c1L)*cos(q(1,loop)),'.k','MarkerSize',max([ms2L*msmul 1]))
        plot((a2L)*sin(q(1,loop)),height_adj-(a2L)*cos(q(1,loop)),'.k','MarkerSize',max([mt1L*mtmul 1]))
        plot((a2L+b2L)*sin(q(1,loop)),height_adj-(a2L+b2L)*cos(q(1,loop)),'.k','MarkerSize',max([mt2L*mtmul 1]))
        
        % Plot Ramp
        plot([LL*sin(q(1,loop))-2*cos(theta), LL*sin(q(1,loop))+2*cos(theta)], ...
            height_adj+[-LL*cos(q(1,loop))+2*sin(theta), -LL*cos(q(1,loop))-2*sin(theta)],...
            '-k','LineWidth',4)
		
		% Anotations
        text(-.2,.9,'Left Stance')
        text(-0.4, 0.9, num2str(loop))

		% Update Plot
        drawnow
		hold off

        
        % Save each individual frame to file for later video generation
        if(save_walking_figure == true) 
            set(gca,'XTick',[])
            set(gca,'YTick',[])
            set(gcf, 'PaperPosition', [1 1 5.599 4.0]);
            print('-djpeg','-r300',strcat(['~/Desktop/walking/' num2str(print_count) '.jpg']))
            print_count = print_count + 1;
        end
        
    end
end



%ENERGY PLOT
if(energy && exist('E_cont'))
    figure(5)
    plot(E_cont,'*b')
    hold on
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    COLLISION EVENT    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%NOTE:  
%    "p" = post collision
%    "m" = pre collision

%HEEL STRIKE
%Heel strike angle
alpha = q(1,end) - q(2,end);


% Pre-colllision angular velocity
qdm = [qd(1,end); qd(2,end)];

% Angular positions
q1 = q(1,end);
q2 = q(2,end);

%Angular Momentums - Pre collision
Qm11 = LL*mt2R*cos(q1 - q2)*(a1R + b1R + c1R + c2R) + mt1L*cos(q1)*(LR*cos(q2) - a2L*cos(q1))*(a1L + b1L + b2L + c1L + c2L) + LL*mt1R*cos(q1 - q2)*(a1R + b1R + b2R + c1R + c2R) + LL*LR*mh*cos(q1 - q2) + mt1L*sin(q1)*(LR*sin(q2) - a2L*sin(q1))*(a1L + b1L + b2L + c1L + c2L) - mt2L*cos(q1)*(cos(q1)*(a2L + b2L) - LR*cos(q2))*(a1L + b1L + c1L + c2L) - mt2L*sin(q1)*(sin(q1)*(a2L + b2L) - LR*sin(q2))*(a1L + b1L + c1L + c2L);
Qm12 = - a2R*mt1R*(a1R + b1R + b2R + c1R + c2R) - mt2R*(a2R + b2R)*(a1R + b1R + c1R + c2R);
Qm21 = - a2L*mt1L*(a1L + b1L + b2L + c1L + c2L) - mt2L*(a2L + b2L)*(a1L + b1L + c1L + c2L);
Qm22 = 0;

Qm2=[Qm11, Qm12; Qm21, Qm22];


%Angular Momentums - Post collision
Qp11 = a2L^2*mt1L + a2L^2*mt2L + b2L^2*mt2L + 2*a2L*b2L*mt2L - LR*a2L*mt1L*cos(q1 - q2) - LR*a2L*mt2L*cos(q1 - q2) - LR*b2L*mt2L*cos(q1 - q2);
Qp12 = mt2R*(a1R + b1R + c1R + c2R)^2 + mt1R*(a1R + b1R + b2R + c1R + c2R)^2 + LR^2*mh - LR*mt2L*cos(q2)*(cos(q1)*(a2L + b2L) - LR*cos(q2)) - LR*mt2L*sin(q2)*(sin(q1)*(a2L + b2L) - LR*sin(q2)) + LR*mt1L*cos(q2)*(LR*cos(q2) - a2L*cos(q1)) + LR*mt1L*sin(q2)*(LR*sin(q2) - a2L*sin(q1));
Qp21 = mt2L*(a2L + b2L)^2 + a2L^2*mt1L;
Qp22 = -LR*cos(q1 - q2)*(a2L*mt1L + a2L*mt2L + b2L*mt2L);

Qp2 = [Qp12, Qp11; Qp22, Qp21];


% Post-collision angular velocity
qdp = Qp2^(-1)*Qm2*qdm;


% Walker goes into 2-link right left
q=q(:,end);
