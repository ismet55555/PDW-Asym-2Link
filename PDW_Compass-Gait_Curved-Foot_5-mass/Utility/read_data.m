clc
close all
clear all


tic

%Read data
data = dlmread('DATA.txt');

%Select only successfuly stable gaits
hit = data(data(:,22) == 1, :);

CollectedDataSize = size(data)
StableGait = size(hit)

%Max = [max(data(:,1)) max(data(:,2)) max(data(:,3)) max(data(:,4)) max(data(:,5))]
%Min = [min(data(:,1)) min(data(:,2)) min(data(:,3)) min(data(:,4)) min(data(:,5))]


% figure
% plot3( data(:,2), data(:,4), data(:,5), '.y')
% hold on
% plot3(hit(:,2), hit(:,4), hit(:,5), 'ok', 'markersize', 5, 'markerfacecolor', 'k')
% %axis([min(data(:,2)) max(data(:,2)) min(data(:,4)) max(data(:,4)) min(data(:,5)) max(data(:,5))])
% xlabel('q2');  ylabel('qd2');  zlabel('ramp')
% 
% %Plot mean
% plot3(mean(hit(:,2)), mean(hit(:,4)), mean(hit(:,5)), 'om', 'markersize', 10, 'markerfacecolor', 'm')


% figure
% plot3(hit(:,1), hit(:,3), hit(:,5), '.k')
% hold on
% plot3(mean(hit(:,1)), mean(hit(:,3)), mean(hit(:,5)), 'om', 'markersize', 10, 'markerfacecolor', 'm')
% plot3([(mean(hit(:,1))-std(hit(:,1))), (mean(hit(:,1))+std(hit(:,1)))], ...
%     [(mean(hit(:,3))-std(hit(:,3))), (mean(hit(:,3))+std(hit(:,3)))], ...
%     [(mean(hit(:,5))-std(hit(:,5))), (mean(hit(:,5))+std(hit(:,5)))], ...
%     ':m', 'linewidth', 3)
% grid on
% xlabel('q1');   ylabel('qd1'); zlabel('ramp')


% figure
% plot3(hit(:,2), hit(:,4), hit(:,5), '.k')
% hold on
% plot3(mean(hit(:,2)), mean(hit(:,4)), mean(hit(:,5)), 'om', 'markersize', 10, 'markerfacecolor', 'm')
% plot3([(mean(hit(:,2))-std(hit(:,2))), (mean(hit(:,2))+std(hit(:,2)))], ...
%     [(mean(hit(:,4))-std(hit(:,4))), (mean(hit(:,4))+std(hit(:,4)))], ...
%     [(mean(hit(:,5))-std(hit(:,5))), (mean(hit(:,5))+std(hit(:,5)))], ...
%     ':m', 'linewidth', 3)
% grid on
% xlabel('q2');   ylabel('qd2'); zlabel('ramp')


% %Finding averages and standard deviations of angle and angular velocities
% means = [mean(hit(:,1)), mean(hit(:,2)), mean(hit(:,3)), mean(hit(:,4)), mean(hit(:,5))]
% means_m_std = [mean(hit(:,1))-std(hit(:,1)), mean(hit(:,2))-std(hit(:,2)), ...
%                mean(hit(:,3))-std(hit(:,3)), mean(hit(:,4))-std(hit(:,4)), ...
%                mean(hit(:,5))-std(hit(:,5))]
% means_p_std = [mean(hit(:,1))+std(hit(:,1)), mean(hit(:,2))+std(hit(:,2)), ...
%                mean(hit(:,3))+std(hit(:,3)), mean(hit(:,4))+std(hit(:,4)), ...
%                mean(hit(:,5))+std(hit(:,5))]

      
           
           
          







%Plotting foot change rate vs vertical foot reaction force
figure
hit_1 = hit(hit(:,5) == 0.0650, :);
hit_2 = hit(hit(:,5) == 0.0500, :);
hit_3 = hit(hit(:,5) == 0.0350, :);

%Regression Fits
x = linspace(-0.19, 0, length(hit_1(:,10)));
one     = (hit_1(:,15)+hit_1(:,17))/2;   %normalize 
two     = (hit_2(:,15)+hit_2(:,17))/2;
three   = (hit_3(:,15)+hit_3(:,17))/2;
maxi = max([one; two; three]);


p_hit_1 = polyfit(hit_1(:,10), one./maxi, 2);
p_hit_2 = polyfit(hit_2(:,10), two./maxi, 2);
p_hit_3 = polyfit(hit_3(:,10), three./maxi, 2);
pv_hit_1 = polyval(p_hit_1, x);
pv_hit_2 = polyval(p_hit_2, x);
pv_hit_3 = polyval(p_hit_3, x);

plot(hit_1(:,10), one./maxi, '.k')
hold on
plot(hit_2(:,10), two./maxi, '.b')
plot(hit_3(:,10), three./maxi, '.r')

plot(x, pv_hit_1, '--k')
plot(x, pv_hit_2, '--b')
plot(x, pv_hit_3, '--r')

grid on
xlabel('Radius Change (rLb, rRb) (Meters)','FontSize',12, 'FontName','Times New Roman');   
ylabel('Vertical Foot Reaction Force (Newtons)','FontSize',12, 'FontName','Times New Roman')           
set(gca,'FontName','Times New Roman', 'FontSize',12);
legend('Ramp = 0.065', 'Ramp = 0.050', 'Ramp = 0.035')
title('PDW Maximum Vertical Ground Reaction Force', 'FontSize',14, 'FontName','Times New Roman');
h_legend = legend();
set(h_legend,'FontSize',10, 'FontName','Times New Roman');
set(gcf,'position', [771   614   711   315])








%Plotting foot change rate vs horizontal foot reaction force
figure
hit_1 = hit(hit(:,5) == 0.0650, :);
hit_2 = hit(hit(:,5) == 0.0500, :);
hit_3 = hit(hit(:,5) == 0.0350, :);

%Regression Fits
x = linspace(-0.19, 0, length(hit_1(:,10)));
one     = (hit_1(:,16)+hit_1(:,18))/2;   %normalize 
two     = (hit_2(:,16)+hit_2(:,18))/2;
three   = (hit_3(:,16)+hit_3(:,18))/2;
maxi = max([one; two; three]);

p_hit_1 = polyfit(hit_1(:,10), one./maxi, 2);
p_hit_2 = polyfit(hit_2(:,10), two./maxi, 2);
p_hit_3 = polyfit(hit_3(:,10), three./maxi, 2);
pv_hit_1 = polyval(p_hit_1, x);
pv_hit_2 = polyval(p_hit_2, x);
pv_hit_3 = polyval(p_hit_3, x);

plot(hit_1(:,10), one./maxi, '.k')
hold on
plot(hit_2(:,10), two./maxi, '.b')
plot(hit_3(:,10), three./maxi, '.r')

plot(x, pv_hit_1, '--k')
plot(x, pv_hit_2, '--b')
plot(x, pv_hit_3, '--r')

grid on
xlabel('Radius Change (rLb, rRb) (Meters)','FontSize',12, 'FontName','Times New Roman');   
ylabel('Horizontal Foot Reaction Force (Newtons)','FontSize',12, 'FontName','Times New Roman')           
set(gca,'FontName','Times New Roman', 'FontSize',12);
legend('Ramp = 0.065', 'Ramp = 0.050', 'Ramp = 0.035')
title('PDW Maximum Horizontal Ground Reaction Force', 'FontSize',14, 'FontName','Times New Roman');
h_legend = legend();
set(h_legend,'FontSize',10, 'FontName','Times New Roman');
set(gcf,'position', [771   614   711   315])
% toc



% figure
% plot(hit_1(:,10), (hit_1(:,19)+hit_1(:,20))/2, '.k')
% hold on
% plot(hit_1(:,10), hit_1(:,14), '.b')
% 
% axis([-0.2 0 0 1.8])
% grid on





%Plotting foot change rate vs work/meter walked
figure
hit_1 = hit(hit(:,5) == 0.0650, :);
hit_2 = hit(hit(:,5) == 0.0500, :);
hit_3 = hit(hit(:,5) == 0.0350, :);

%Regression Fits
x = linspace(-0.19, 0, length(hit_1(:,10)));
one     = ((hit_1(:,19)+hit_1(:,20))/2)./hit_1(:,14);   %normalize 
two     = ((hit_2(:,19)+hit_2(:,20))/2)./hit_2(:,14);
three   = ((hit_3(:,19)+hit_3(:,20))/2)./hit_3(:,14);
maxi = max([one; two; three]);

p_hit_1 = polyfit(hit_1(:,10), one./maxi, 2);
p_hit_2 = polyfit(hit_2(:,10), two./maxi, 2);
p_hit_3 = polyfit(hit_3(:,10), three./maxi, 2);
pv_hit_1 = polyval(p_hit_1, x);
pv_hit_2 = polyval(p_hit_2, x);
pv_hit_3 = polyval(p_hit_3, x);

plot(hit_1(:,10), one./maxi, '.k')
hold on
plot(hit_2(:,10), two./maxi, '.b')
plot(hit_3(:,10), three./maxi, '.r')

plot(x, pv_hit_1, '--k')
plot(x, pv_hit_2, '--b')
plot(x, pv_hit_3, '--r')

grid on
xlabel('Radius Change (rLb, rRb) (Meters)','FontSize',12, 'FontName','Times New Roman');   
ylabel('Work/Distance (Joules/Meter)','FontSize',12, 'FontName','Times New Roman')   
set(gca,'FontName','Times New Roman', 'FontSize',12);
legend('Ramp = 0.065', 'Ramp = 0.050', 'Ramp = 0.035')
title('PDW Walking Energy Expendature', 'FontSize',14, 'FontName','Times New Roman');
%h_legend = legend();
%set(h_legend,'FontSize',10, 'FontName','Times New Roman');
set(gcf,'position', [771   614   711   315])




% figure
% plot((hit_1(:,19)+hit_1(:,20))/2, hit_1(:,14), '.b')
% grid on

% for i = 1:length(hit(:,1))
%     fid = fopen('hit.txt','a');
%     fprintf(fid, '%6.4f\t%6.4f\t%6.4f\t%6.4f\t%6.4f\t%6.4f\t%6.4f\t%6.4f\t%6.4f\t%6.4f\t%6.4f\t\r\n', ...
%         hit(i,1),  hit(i,2), hit(i,3), hit(i,4), hit(i,5), hit(i,6), hit(i,7),  hit(i,8), hit(i,9), hit(i,10), hit(i,11) );
%     fclose(fid);
% end

