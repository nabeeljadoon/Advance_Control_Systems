%Nabeel Ahmad Khan - Mechatronics%
A = [0 1; 0 -4.919];
B = [0; -2.474];
G = [-6.4673 0.3715];
AA = A - B*G;
BB = [-20;0];
[x,z,t] = step(AA,BB,AA,BB);
x1 = [1 0]*x';
plot(t,x1); grid
title('x1(theta) versus t')
xlabel('t sec')
ylabel('x1 = theta')
