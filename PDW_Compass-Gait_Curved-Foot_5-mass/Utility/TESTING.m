clear all
close all
clc
mfilename

global log


log = logger(true, true, true, false, false, "cool_log.log")


blah()

tic
log.info("cool")
toc

tic
log.warning("yo")
toc

log.help


return












figure
plot(NaN, NaN)
hold on
grid on
box on


% x = results.motion.all.left.q;
% y = results.motion.all.left.qd;
% plot(x, y, '-b')
% 
% x = results.motion.all.right.q;
% y = results.motion.all.right.qd;
% plot(x, y, '-r')




% x = results.motion.right.q.data_interp{2:3};
% y = results.motion.right.qd.data_interp{2:3};
% plot(x, y, '.b')




x = results.sim.time;
y = results.motion.all.right.qd.data;
plot(x, y, '.b')

x = results.collision.heel.all.right.time_abs;
y = results.collision.heel.all.right.qd_pre;
plot(x, y, '*b')


x = results.sim.time;
y = results.motion.all.left.qd.data;
plot(x, y, '.r')

x = results.collision.heel.all.left.time_abs;
y = results.collision.heel.all.left.qd_pre;
plot(x, y, '*r')


% 
% x = results.sim.time;
% y = results.motion.all.left.qd.data;
% plot(x, y, '-r')
% results.motion.left.qd.mean



function blah()
    blah2()
end

function blah2()
    global log
    log.error("yo")
end


