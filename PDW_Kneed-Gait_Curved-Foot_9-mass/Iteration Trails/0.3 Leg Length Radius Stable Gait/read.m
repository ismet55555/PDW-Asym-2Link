clc
close all
clear all


tic

%Read data
data = dlmread('DATA5.txt');

%Select only successfuly stable gaits
hit = data(data(:,6) == 1, :);

CollectedDataSize = size(data)
StableGait = size(hit)

%Max = [max(data(:,1)) max(data(:,2)) max(data(:,3)) max(data(:,4)) max(data(:,5))]
%Min = [min(data(:,1)) min(data(:,2)) min(data(:,3)) min(data(:,4)) min(data(:,5))]

figure
plot3( data(:,2), data(:,4), data(:,5), '.y')
hold on
plot3(hit(:,2), hit(:,4), hit(:,5), 'ok', 'markersize', 5, 'markerfacecolor', 'k')
axis([min(data(:,2)) max(data(:,2)) min(data(:,4)) max(data(:,4)) min(data(:,5)) max(data(:,5))])
xlabel('q2');  ylabel('qd2');  zlabel('ramp')

%Plot mean
plot3(mean(hit(:,2)), mean(hit(:,4)), mean(hit(:,5)), 'om', 'markersize', 10, 'markerfacecolor', 'm')


figure
plot3(hit(:,1), hit(:,3), hit(:,5), '.k')
hold on
plot3(mean(hit(:,1)), mean(hit(:,3)), mean(hit(:,5)), 'om', 'markersize', 10, 'markerfacecolor', 'm')
plot3([(mean(hit(:,1))-std(hit(:,1))), (mean(hit(:,1))+std(hit(:,1)))], ...
    [(mean(hit(:,3))-std(hit(:,3))), (mean(hit(:,3))+std(hit(:,3)))], ...
    [(mean(hit(:,5))-std(hit(:,5))), (mean(hit(:,5))+std(hit(:,5)))], ...
    ':m', 'linewidth', 3)
grid on
xlabel('q1');   ylabel('qd1'); zlabel('ramp')

figure
plot3(hit(:,2), hit(:,4), hit(:,5), '.k')
hold on
plot3(mean(hit(:,2)), mean(hit(:,4)), mean(hit(:,5)), 'om', 'markersize', 10, 'markerfacecolor', 'm')
plot3([(mean(hit(:,2))-std(hit(:,2))), (mean(hit(:,2))+std(hit(:,2)))], ...
    [(mean(hit(:,4))-std(hit(:,4))), (mean(hit(:,4))+std(hit(:,4)))], ...
    [(mean(hit(:,5))-std(hit(:,5))), (mean(hit(:,5))+std(hit(:,5)))], ...
    ':m', 'linewidth', 3)
grid on
xlabel('q2');   ylabel('qd2'); zlabel('ramp')


means = [mean(hit(:,1)), mean(hit(:,2)), mean(hit(:,3)), mean(hit(:,4)), mean(hit(:,5))]
means_m_std = [mean(hit(:,1))-std(hit(:,1)), mean(hit(:,2))-std(hit(:,2)), ...
               mean(hit(:,3))-std(hit(:,3)), mean(hit(:,4))-std(hit(:,4)), ...
               mean(hit(:,5))-std(hit(:,5))]
means_p_std = [mean(hit(:,1))+std(hit(:,1)), mean(hit(:,2))+std(hit(:,2)), ...
               mean(hit(:,3))+std(hit(:,3)), mean(hit(:,4))+std(hit(:,4)), ...
               mean(hit(:,5))+std(hit(:,5))]


toc




% for i = 1:length(hit(:,1))
%     fid = fopen('hit.txt','a');
%     fprintf(fid, '%6.4f\t%6.4f\t%6.4f\t%6.4f\t%6.4f\t%6.4f\t\r\n', hit(i,1),  hit(i,2), hit(i,3), hit(i,4), hit(i,5), hit(i,6));
%     fclose(fid);
% end

