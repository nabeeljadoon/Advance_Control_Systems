%Nabeel Ahmad Khan - Mechatronics%
A = [0 1; 0 -5.345];
B = [0; 0.3566];
G = [44.8682 -3.7717];
AA = A - B*G;
BB = [-20;0];
[x,z,t] = step(AA,BB,AA,BB);
x1 = [1 0]*x';
plot(t,x1); grid
title('x1(theta) versus t')
xlabel('t sec')
ylabel('x1 = theta')
