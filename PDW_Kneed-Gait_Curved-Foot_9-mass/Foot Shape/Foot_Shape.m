clc
clear all
close all



q1 = linspace(pi/2, -pi/2, 70);

rA = 1/3;
rB = 0.19;
r = rA  - rB*(q1);

figure(1)
plot(q1, r, '--r')
grid on
xlabel('Angle');  ylabel('Radius')


for i = -0.19 : 0.0316667 : 0.19
    %i = -0.095;
    r = rA - i*q1;
    
    figure(2)
    polar(q1-pi/2, r, '--r')
    hold on
    pause(0.5)
end
