%TWO LINK MECHANISM
%RIGHT

function [q, qdp, stop_condition] = two_link_right_masses(q,qd,parameters,params)

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

time(1) = 0;    %starting time

x_heel = -sin(q(1,end))*LR + sin(q(2,end))*LL;
y_heel = abs(cos(q(1,end))*LR) - abs(cos(q(2,end))*LL);
ramp = (tan(theta)*x_heel);

while(y_heel >= ramp || (qd(2,count-1) > 0))  
    x_heel = -sin(q(1,count-1))*LR + sin(q(2,count-1))*LL;
    y_heel = abs(cos(q(1,count-1))*LR) - abs(cos(q(2,count-1))*LL);
    ramp = -(tan(theta)*x_heel);  
    
    if (qd(2,count-1) > 0)     
        ramp = ramp - 5;  
    end
    
%     fprintf('============== %i ===============\n', count-1)
%     fprintf('X1: %2.5f   X2: %2.5f   X_heel: %2.5f\n', sin(q(1,count-1))*LL, sin(q(2,count-1))*LR, x_heel);
%     fprintf('Y1: %2.5f   Y2: %2.5f   y_heel: %2.5f\n', cos(q(1,count-1))*LL, cos(q(2,count-1))*LR, y_heel);
%     fprintf('SWING SPEED: %2.5f  RAMP: %2.5f\n', qd(2,count-1), ramp);
    
    
%>--------------------------------DYNAMICS---------------------------- 

    %     q1 = q(1,count-1);    q2 = q(2,count-1);

    %     M11 = mt2R*(a1R + b1R + c1R + c2R)^2 + ms2R*(a1R + b1R)^2 + mt1R*(a1R + b1R + b2R + c1R + c2R)^2 + LR^2*mh + LR^2*ms1L + LR^2*ms2L + LR^2*mt1L + LR^2*mt2L + a1R^2*ms1R;
    %     M12 = -LR*cos(q1 - q2)*(a2L*ms1L + a2L*ms2L + a2L*mt1L + a2L*mt2L + b1L*ms1L + b2L*ms1L + b2L*ms2L + b2L*mt2L + c1L*ms1L + c1L*ms2L + c2L*ms1L + c2L*ms2L);
    %     M21 = -LR*cos(q1 - q2)*(a2L*ms1L + a2L*ms2L + a2L*mt1L + a2L*mt2L + b1L*ms1L + b2L*ms1L + b2L*ms2L + b2L*mt2L + c1L*ms1L + c1L*ms2L + c2L*ms1L + c2L*ms2L);
    %     M22 = ms2L*(a2L + b2L + c1L + c2L)^2 + mt2L*(a2L + b2L)^2 + ms1L*(a2L + b1L + b2L + c1L + c2L)^2 + a2L^2*mt1L;

    %     N12 = -LR*sin(q1 - q2)*(a2L*ms1L + a2L*ms2L + a2L*mt1L + a2L*mt2L + b1L*ms1L + b2L*ms1L + b2L*ms2L + b2L*mt2L + c1L*ms1L + c1L*ms2L + c2L*ms1L + c2L*ms2L);
    %     N21 = LR*sin(q1 - q2)*(a2L*ms1L + a2L*ms2L + a2L*mt1L + a2L*mt2L + b1L*ms1L + b2L*ms1L + b2L*ms2L + b2L*mt2L + c1L*ms1L + c1L*ms2L + c2L*ms1L + c2L*ms2L);

    %     G11 = -g*sin(q1)*(LR*mh + LR*ms1L + LR*ms2L + LR*mt1L + LR*mt2L + a1R*ms1R + a1R*ms2R + a1R*mt1R + a1R*mt2R + b1R*ms2R + b1R*mt1R + b1R*mt2R + b2R*mt1R + c1R*mt1R + c1R*mt2R + c2R*mt1R + c2R*mt2R);
    %     G21 = g*(a2L*mt1L*sin(q2) + ms2L*sin(q2)*(a2L + b2L + c1L + c2L) + mt2L*sin(q2)*(a2L + b2L) + ms1L*sin(q2)*(a2L + b1L + b2L + c1L + c2L));

    %     H = [M11, M12; M21, M22];
    %     B = [0 N12;N21 0];
    %     G = [G11;G21];

    %     qdd2 = inv(H)*(-B*qd(1:2,count-1).^2-G);

%>---------- NEW  DYNAMICS - NO SHANK MASS --------<
    
    q1 = q(1,count-1);    q2 = q(2,count-1);

    M11 = mt2R*(a1R + b1R + c1R + c2R)^2 + mt1R*(a1R + b1R + b2R + c1R + c2R)^2 + LR^2*mh + LR^2*mt1L + LR^2*mt2L;
    M12 = -LR*cos(q1 - q2)*(a2L*mt1L + a2L*mt2L + b2L*mt2L);
    M21 = -LR*cos(q1 - q2)*(a2L*mt1L + a2L*mt2L + b2L*mt2L);
    M22 = mt2L*(a2L + b2L)^2 + a2L^2*mt1L;

    N12 = -LR*sin(q1 - q2)*(a2L*mt1L + a2L*mt2L + b2L*mt2L);
    N21 = LR*sin(q1 - q2)*(a2L*mt1L + a2L*mt2L + b2L*mt2L);

    G11 = -g*(mt2R*sin(q1)*(a1R + b1R + c1R + c2R) + mt1R*sin(q1)*(a1R + b1R + b2R + c1R + c2R) + LR*mh*sin(q1) + LR*mt1L*sin(q1) + LR*mt2L*sin(q1));
    G21 = g*sin(q2)*(a2L*mt1L + a2L*mt2L + b2L*mt2L);

    H = [M11, M12; M21, M22];
    B = [0 N12;N21 0];
    G = [G11;G21];
    
    qdd2 = inv(H)*(-B*qd(1:2,count-1).^2-G);
    
    %Numeric integration to find qd and q
    qdd(:,count) = qdd2;
    qd(:,count) = qd(:,count-1) + qdd(:,count)*dt;
    q(:,count) = q(:,count-1) + qd(:,count)*dt;
    
    time(count) = time(count-1) + dt; %time step for numeric integration
        
    
    %calling engergy function to check if energy is right 
    %Energy plots are inside this
    E_cont(count) = energy_right_masses([q(1,end);q(2,end);qd(2,end)],[qd(1,end);qd(2,end);qd(2,end)],parameters,params);
       
    count = count + 1;
    
    %if the loop exceeds some iteration - stop
    if(count > 1500000)
        stop_condition = 1;
        break;
    end
end

% fprintf('\nq1: %g\n',  q(1, count - 1))
% fprintf('q2: %g\n\n',  q(2, count - 1))
% 
% fprintf('qd1: %g\n', qd(1, count - 1))
% fprintf('qd2: %g\n',  qd(2, count - 1))
% 
% pause

%limit cycle plot
if (cycle)
    figure(127)
    plot(q(1,:),qd(1,:),'k-')
    hold on
    plot(q(1,1),qd(1,1),'*m')
end

if(length(q) < 4) %stopping condition check to make sure it ran more then tow iterations
    disp('WARNING: two_link_model_right_PDW ran less than two iterations')
    stop_condition = 1;
elseif(display)
    
    figure(999)
    
    %Plotting of simulation
    for loop = 1:100:length(q)
        
        height_adj = LR*cos(q(1,loop));
        
        plot([0 LR*sin(q(1,loop))],height_adj+[0 -LR*cos(q(1,loop))],'-b','LineWidth',2)
        axis([-.5 .5 -.05 1.25])
        hold on
        plot([0 LL*sin(q(2,loop))],height_adj+[0 -LL*cos(q(2,loop))],'-r','LineWidth',2)
        
        plot(a2L*sin(q(2,loop)),height_adj-a2L*cos(q(2,loop)),'.k','MarkerSize',max([mt1L*mtmul 1]))
        plot((a2L+b2L)*sin(q(2,loop)),height_adj-(a2L+b2L)*cos(q(2,loop)),'.k','MarkerSize',max([mt2L*mtmul 1]))
        plot(0,height_adj,'.k','MarkerSize',max([mh*mhmul 1]))
        plot(ltL*sin(q(2,loop))+c1L*sin(q(2,loop)),height_adj-ltL*cos(q(2,loop))-c1L*cos(q(2,loop)),'.k','MarkerSize',max([ms2L*msmul 1]))
        plot(ltL*sin(q(2,loop))+(c1L+b1L)*sin(q(2,loop)),height_adj-ltL*cos(q(2,loop))-(c1L+b1L)*cos(q(2,loop)),'.k','MarkerSize',max([ms1L*msmul 1]))
        plot((ltR+c1R+b1R)*sin(q(1,loop)),height_adj-(ltR+c1R+b1R)*cos(q(1,loop)),'.k','MarkerSize',max([ms1R*msmul 1]))
        plot((ltR+c1R)*sin(q(1,loop)),height_adj-(ltR+c1R)*cos(q(1,loop)),'.k','MarkerSize',max([ms2R*msmul 1]))
        plot((a2R)*sin(q(1,loop)),height_adj-(a2R)*cos(q(1,loop)),'.k','MarkerSize',max([mt1R*mtmul 1]))
        plot((a2R+b2R)*sin(q(1,loop)),height_adj-(a2R+b2R)*cos(q(1,loop)),'.k','MarkerSize',max([mt2R*mtmul 1]))
        
        % drawing the ramp
        plot([LR*sin(q(1,loop))-2*cos(theta) LR*sin(q(1,loop))+2*cos(theta)],...
            height_adj+[-LR*cos(q(1,loop))+2*sin(theta) -LR*cos(q(1,loop))-2*sin(theta)],...
            '-k','LineWidth',4)
        text(-.2,.9,'right stance');
        text(-0.4, 0.9, num2str(loop))
        
       
        grid on
        
        drawnow
        hold off
        
        if(save_walking_figure == true) %VIDEO ANIMATION
            set(gca,'XTick',[])
            set(gca,'YTick',[])
            set(gcf, 'PaperPosition', [1 1 5.599 4.0]);
            print('-djpeg','-r300',strcat(['~/Desktop/walking/' num2str(print_count) '.jpg']))
            print_count = print_count + 1;
        end
    end
end

%ENERGY PLOT
if(energy & exist('E_cont'))
    figure(5)
    plot(E_cont,'*c')
    hold on
end


%=====================    COLLISION EVENTS    ========================

%HEEL STRIKE

%Heel strike angle
alpha=q(1,end)-q(2,end);

qdm=[qd(1,end);qd(2,end)];


%NOTE:  "p"=post collision, "m" = pre collision  (plus/minus in chen)

%Angular Momentums

%Volocities right after knee strike (New)
%Angular Momentums
q1 = q(1,end);    q2 = q(2,end);

%Pre collision
% Qm11= LR*mt2L*cos(q1 - q2)*(a1L + b1L + c1L + c2L) + LR*ms2L*cos(q1 - q2)*(a1L + b1L) - a1R*ms1R*cos(q1)*(cos(q1)*(a2R + b1R + b2R + c1R + c2R) - LL*cos(q2)) + mt1R*cos(q1)*(LL*cos(q2) - a2R*cos(q1))*(a1R + b1R + b2R + c1R + c2R) + LR*mt1L*cos(q1 - q2)*(a1L + b1L + b2L + c1L + c2L) + LL*LR*mh*cos(q1 - q2) + LR*a1L*ms1L*cos(q1 - q2) - a1R*ms1R*sin(q1)*(sin(q1)*(a2R + b1R + b2R + c1R + c2R) - LL*sin(q2)) + mt1R*sin(q1)*(LL*sin(q2) - a2R*sin(q1))*(a1R + b1R + b2R + c1R + c2R) - ms2R*cos(q1)*(a1R + b1R)*(cos(q1)*(a2R + b2R + c1R + c2R) - LL*cos(q2)) - mt2R*cos(q1)*(cos(q1)*(a2R + b2R) - LL*cos(q2))*(a1R + b1R + c1R + c2R) - ms2R*sin(q1)*(sin(q1)*(a2R + b2R + c1R + c2R) - LL*sin(q2))*(a1R + b1R) - mt2R*sin(q1)*(sin(q1)*(a2R + b2R) - LL*sin(q2))*(a1R + b1R + c1R + c2R);
% Qm12= - a1L*ms1L*(a2L + b1L + b2L + c1L + c2L) - a2L*mt1L*(a1L + b1L + b2L + c1L + c2L) - ms2L*(a1L + b1L)*(a2L + b2L + c1L + c2L) - mt2L*(a2L + b2L)*(a1L + b1L + c1L + c2L);
% Qm21= - a1R*ms1R*(a2R + b1R + b2R + c1R + c2R) - a2R*mt1R*(a1R + b1R + b2R + c1R + c2R) - ms2R*(a1R + b1R)*(a2R + b2R + c1R + c2R) - mt2R*(a2R + b2R)*(a1R + b1R + c1R + c2R);
% Qm22= 0;
% 
% Qm2=[Qm11,Qm12;Qm21,Qm22];

%>----- NEW - no shank mass ---------<
Qm11= LR*mt2L*cos(q1 - q2)*(a1L + b1L + c1L + c2L) + mt1R*cos(q1)*(LL*cos(q2) - a2R*cos(q1))*(a1R + b1R + b2R + c1R + c2R) + LR*mt1L*cos(q1 - q2)*(a1L + b1L + b2L + c1L + c2L) + LL*LR*mh*cos(q1 - q2) + mt1R*sin(q1)*(LL*sin(q2) - a2R*sin(q1))*(a1R + b1R + b2R + c1R + c2R) - mt2R*cos(q1)*(cos(q1)*(a2R + b2R) - LL*cos(q2))*(a1R + b1R + c1R + c2R) - mt2R*sin(q1)*(sin(q1)*(a2R + b2R) - LL*sin(q2))*(a1R + b1R + c1R + c2R);
Qm12= - a2L*mt1L*(a1L + b1L + b2L + c1L + c2L) - mt2L*(a2L + b2L)*(a1L + b1L + c1L + c2L);
Qm21= - a2R*mt1R*(a1R + b1R + b2R + c1R + c2R) - mt2R*(a2R + b2R)*(a1R + b1R + c1R + c2R);
Qm22= 0;

Qm2=[Qm11,Qm12;Qm21,Qm22];

%Post collision
% Qp11= mt2R*sin(q1)*(a2R + b2R)*(sin(q1)*(a2R + b2R) - LL*sin(q2)) + ms1R*cos(q1)*(cos(q1)*(a2R + b1R + b2R + c1R + c2R) - LL*cos(q2))*(a2R + b1R + b2R + c1R + c2R) - a2R*mt1R*cos(q1)*(LL*cos(q2) - a2R*cos(q1)) + ms1R*sin(q1)*(sin(q1)*(a2R + b1R + b2R + c1R + c2R) - LL*sin(q2))*(a2R + b1R + b2R + c1R + c2R) + ms2R*cos(q1)*(cos(q1)*(a2R + b2R + c1R + c2R) - LL*cos(q2))*(a2R + b2R + c1R + c2R) - a2R*mt1R*sin(q1)*(LL*sin(q2) - a2R*sin(q1)) + ms2R*sin(q1)*(sin(q1)*(a2R + b2R + c1R + c2R) - LL*sin(q2))*(a2R + b2R + c1R + c2R) + mt2R*cos(q1)*(a2R + b2R)*(cos(q1)*(a2R + b2R) - LL*cos(q2));
% Qp12= mt2L*(a1L + b1L + c1L + c2L)^2 + ms2L*(a1L + b1L)^2 + mt1L*(a1L + b1L + b2L + c1L + c2L)^2 + LL^2*mh + a1L^2*ms1L - LL*ms2R*sin(q2)*(sin(q1)*(a2R + b2R + c1R + c2R) - LL*sin(q2)) - LL*mt2R*cos(q2)*(cos(q1)*(a2R + b2R) - LL*cos(q2)) - LL*ms1R*cos(q2)*(cos(q1)*(a2R + b1R + b2R + c1R + c2R) - LL*cos(q2)) - LL*mt2R*sin(q2)*(sin(q1)*(a2R + b2R) - LL*sin(q2)) + LL*mt1R*cos(q2)*(LL*cos(q2) - a2R*cos(q1)) - LL*ms1R*sin(q2)*(sin(q1)*(a2R + b1R + b2R + c1R + c2R) - LL*sin(q2)) + LL*mt1R*sin(q2)*(LL*sin(q2) - a2R*sin(q1)) - LL*ms2R*cos(q2)*(cos(q1)*(a2R + b2R + c1R + c2R) - LL*cos(q2));
% Qp21= ms2R*(a2R + b2R + c1R + c2R)^2 + mt2R*(a2R + b2R)^2 + ms1R*(a2R + b1R + b2R + c1R + c2R)^2 + a2R^2*mt1R;
% Qp22= -LL*cos(q1 - q2)*(a2R*ms1R + a2R*ms2R + a2R*mt1R + a2R*mt2R + b1R*ms1R + b2R*ms1R + b2R*ms2R + b2R*mt2R + c1R*ms1R + c1R*ms2R + c2R*ms1R + c2R*ms2R);
% 
% Qp2=[Qp12,Qp11;Qp22,Qp21];

%>----- NEW - no shank mass ---------<
Qp11= a2R^2*mt1R + a2R^2*mt2R + b2R^2*mt2R + 2*a2R*b2R*mt2R - LL*a2R*mt1R*cos(q1 - q2) - LL*a2R*mt2R*cos(q1 - q2) - LL*b2R*mt2R*cos(q1 - q2);
Qp12= mt2L*(a1L + b1L + c1L + c2L)^2 + mt1L*(a1L + b1L + b2L + c1L + c2L)^2 + LL^2*mh - LL*mt2R*cos(q2)*(cos(q1)*(a2R + b2R) - LL*cos(q2)) - LL*mt2R*sin(q2)*(sin(q1)*(a2R + b2R) - LL*sin(q2)) + LL*mt1R*cos(q2)*(LL*cos(q2) - a2R*cos(q1)) + LL*mt1R*sin(q2)*(LL*sin(q2) - a2R*sin(q1));
Qp21= mt2R*(a2R + b2R)^2 + a2R^2*mt1R;
Qp22= -LL*cos(q1 - q2)*(a2R*mt1R + a2R*mt2R + b2R*mt2R);

Qp2=[Qp12,Qp11;Qp22,Qp21];

%fprintf('\nPOST strike\n')
qdp = Qp2^(-1)*Qm2*qdm;

%goes into tree link left
q=q(:,end);


