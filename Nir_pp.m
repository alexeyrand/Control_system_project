clc
close all force
clear variables
clear figures

t = 1:1:50;


% prog = (quatTN0^-1)*quatSN0

x = [25, 0, 2,0,  2];
xk = [30, 0, 30, 0, 18] ;
q0 = cos(x(1)/2)*cos(x(3)/2)*cos(x(5)/2)+sin(x(1)/2)*sin(x(3)/2)*sin(x(5)/2)
q1 = sin(x(1)/2)*cos(x(3)/2)*cos(x(5)/2)-cos(x(1)/2)*sin(x(3)/2)*sin(x(5)/2)
q2 = cos(x(1)/2)*sin(x(3)/2)*cos(x(5)/2)+cos(x(1)/2)*cos(x(3)/2)*sin(x(5)/2)
q3 = cos(x(1)/2)*cos(x(3)/2)*sin(x(5)/2)-sin(x(1)/2)*sin(x(3)/2)*cos(x(5)/2)
x = [30, 0, 30, 0, 18];
k0 = 0.9998;
k1 = 0.5887;
k2 = -0.1928;
k3 = 1;
%S = rk4(t, @newnewnew, S0);
q = [q0, q1, q2, q3]
k = [k0, k1, k2, k3]
q = quaternion(q)
k = quaternion(k)
proh = (q^-1)*k

e1 = 0.079697/sqrt(1-0.076544^2) 
e2 = 1.1953/sqrt(1-0.076544^2) 
e3 = 0.094167/sqrt(1-0.076544^2) 
e = [e1, e2 ,e3]
tetta = 2*acos(0.78162)

%%
S0 = [25, 0, 1, 0, 1, 0.00, q0 ,q1 ,q2, q3];
qnach = quaternion(q0,q1,q2,q3);
qkonech = quaternion(q0,q1,q2,q3);
global u1 u2 u3

% a = linspace(-100, 100, 20);
% 
% for k = 1 : 1 : size(a, 2)
% for q=1:1:size(a, 2)
% X{k, q} = [a(k), a(q)];
% 
% end
% end
% 
% figure(1)
% for i = 1 : 1 : size(X, 2);
% for j = 1 : 1 : size(X, 2);
% 
% [T, Y] = ode45(@newnewnew, [0.5, 0], S0);
% plot(Y(:, 1), Y(:, 2));
% hold on;
% end
% end

[T Y] = ode23(@newnewnew, t, S0);
% [T Y] = ode45(@newnewnew, [0, 200], S0);
figure(), plot(Y(:, 3), Y(:, 4))

figure, plot(T, u1(1, 1 : size(T, 1)));
% hold on;
grid on;
title('Управляющий момент')
% ylim([-200, 200])
figure, plot(T, u2(1, 1 : size(T, 1)));
grid on;
ylabel('u')
xlabel('t')
% ylim([-200, 200])
title('Управляющий момент')
figure, plot(T, u3(1, 1 : size(T, 1)));
grid on;
ylabel('u')
xlabel('t')
title('Управляющий момент')
ylabel('u')
xlabel('t')
% ylim([-100, 100])


figure()
plot(t, Y(:, 1))
hold on;
plot(t, Y(:, 3))
hold on;
plot(t, Y(:, 5))
title('Углы ориентации Эйлера-Крылова во время маневра')
ylabel('угол, град');
xlabel('t, с');
grid on;
grid minor;

figure()
plot(t, Y(:, 2))
hold on;
plot(t, Y(:, 4))
hold on;
plot(t, Y(:, 6))
hold on;
title('Угловые скорости КА во время маневра')
ylabel('Угловая скорость, град/с');
xlabel('t, с');

grid on;
grid minor;

figure()
plot(t, Y(:, 7))
hold on;
plot(t, Y(:, 8))
hold on;
plot(t, Y(:, 9))
hold on;
plot(t, Y(:, 10))
title('Компоненты кватерниона')
ylabel('q');
xlabel('t, с');
grid on;
grid minor;


function [dx] = newnewnew(t, x)
global u1 u2 u3;
 H = 4;
 wrx=1;
 wry=1;
 wrz=1;
 Jx = 200;
 Jy = 230;
 Jz = 97;
iy = Jy/Jz;
iz = Jz/Jx;

% 
A(1, 1) = x(2)*x(1)-x(4)*sin(x(1))-0.02;
A(2, 1) = x(2)*sin(x(1))+x(4)*x(1);
A(3, 1) = -tan(x(3))*(x(2)*x(1)-x(4)*sin(x(1)));
%  
omega1 = -4*( 1*(x(1)-4) +4*(x(2)-H)) ;
omega2 = -1*( 1*(x(3)-2) +4*(x(4)-H)) ;
omega3 = -1*( 1*(x(5)-3) +4*(x(6)-H)) ;


if abs(omega1)> 10
u11=150*sign(omega1);
u1(end + 1) = u11*sign(omega1);
else
    u11=0 ;
    u1(end + 1) = u11;
end

if abs(omega2)>10
u22=150*sign(omega2);
u2(end + 1) = u22*sign(omega2);
else
    u22=0 ;
    u2(end + 1) = u22;
end

if abs(omega3)>10
u33=150*sign(omega3);
u3(end + 1) = u33;
else
    u33=0 ;
    u3(end + 1) = u33*sign(omega3);
end
 %    dx(2, 1) = u1/Iz + (N1/Iz)*r(6) - (N2/Iz)*x(1);
    dx(1, 1) = x(2);
    dx(2, 1) = u11/Jx ;
    dx(3, 1) = x(4);
    dx(4, 1) = u22/Jy;
    dx(5, 1) = x(6);
    dx(6, 1) = u33/Jz;
    
    dx(7, 1) = (-1/2)*(x(2)*x(8)+x(4)*x(9)+x(6)*x(10));
    dx(8, 1) = (1/2)*(x(2)*x(7)+x(4)*x(10)-x(6)*x(9));
    dx(9, 1) = (1/2)*(x(4)*x(7)+x(6)*x(8)-x(2)*x(10));
    dx(10, 1) = (1/2)*(x(6)*x(7)+x(2)*x(9)-x(4)*x(8));

    
end









