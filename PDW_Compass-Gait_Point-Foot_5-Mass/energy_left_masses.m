%ENERGY OF MASSES
%FOR BOTH TWO AND TREE LINK MECHANISMS
%LEFT

function [E] = energy_left_masses(q,qd,parameters,params)

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
mt2L = parameters(15);
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

%Vertical positions (y) potentially for potential energy
yh = LL*cos(q(1));
yms1L = a1L*cos(q(1));
yms2L = (a1L+b1L)*cos(q(1));
yms2R = LL*cos(q(1))-(ltR)*cos(q(2))-c1R*cos(q(3));
yms1R = LL*cos(q(1))-(ltR)*cos(q(2))-(c1R+b1R)*cos(q(3));
ymt1L = (lsL+c2L+b2L)*cos(q(1));
ymt2L = (lsL+c2L)*cos(q(1));
ymt1R = LL*cos(q(1))-a2R*cos(q(2));
ymt2R = LL*cos(q(1))-(a2R+b2R)*cos(q(2));
%y4 = LL*cos(q(1))-ltR*cos(q(2))-b1R*cos(q(3));

%Kinetic Energy (KE = 1/2*mass*position^2*angular velocity^2)
%NOTE = Velocity = Angular Velocity(q)*radius

H1 = ms1L*[(a1L^2) 0 0 ; 0 0 0 ; 0 0 0];
H1a = ms2L*[((a1L+b1L)^2) 0 0 ; 0 0 0 ; 0 0 0];
H2 = mt2L*[((lsL+c2L)^2) 0 0 ; 0 0 0 ; 0 0 0];
H2a = mt1L*[((lsL+b2L+c2L)^2) 0 0 ; 0 0 0 ; 0 0 0];
Hh = mh*[(LL^2) 0 0 ; 0 0 0 ; 0 0 0 ];
H3 = mt1R*[(LL^2)                  (-LL*a2R*cos(q(2)-q(1))) 0 ;
    (-LL*a2R*cos(q(2)-q(1))) (a2R^2)                 0 ; 0 0 0 ];
H3a = mt2R*[(LL^2)                  (-LL*(a2R+b2R)*cos(q(2)-q(1))) 0 ;
    (-LL*(a2R+b2R)*cos(q(2)-q(1))) ((a2R+b2R)^2)                 0 ; 0 0 0 ];
H4 = ms2R*[(LL^2) (-LL*ltR*cos(q(2)-q(1))) (-LL*c1R*cos(q(3)-q(1))) ;
    (-LL*ltR*cos(q(2)-q(1))) (ltR^2) (ltR*c1R*cos(q(3)-q(2))) ;
    (-LL*c1R*cos(q(3)-q(1))) (ltR*c1R*cos(q(3)-q(2))) (c1R^2)];
H4a = ms1R*[(LL^2) (-LL*ltR*cos(q(2)-q(1))) (-LL*(c1R+b1R)*cos(q(3)-q(1))) ;
    (-LL*ltR*cos(q(2)-q(1))) (ltR^2) (ltR*(c1R+b1R)*cos(q(3)-q(2))) ;
    (-LL*(c1R+b1R)*cos(q(3)-q(1))) (ltR*(c1R+b1R)*cos(q(3)-q(2))) ((c1R+b1R)^2)];

H = H1+H2+H3+H4+H1a+H2a+H3a+H4a+Hh;

%KE = .5 * Angular veolcity^T * (mass*position^2) * Angular Velocity
KE = .5*qd'*H*qd;

%PE = gravity * mass * height
PE = g*(ms1L*yms1L+ms2L*yms2L+ ms1R*yms1R+ms2R*yms2R + mt1L*ymt1L+mt2L*ymt2L + mt1R*ymt1R+mt2R*ymt2R+mh*yh);

E = KE + PE;

end