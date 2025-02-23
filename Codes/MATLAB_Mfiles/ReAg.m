%Nabeel Ahmad Khan - Mechatronics%
% We can use MATLAB to design controller gains using pole
% placement. You will enter the desired time constant and damping ratio. 
% The commands: [num,den] = ord2(wn,z), which produces a 
% second-order system, given the natural frequency (wn) and the damping ratio (z). 
% Then we use the denominator (den) to specify the poles; and 
% K = acker(A,B,poles), which calculates controller gains from the system matrix (A), 
% the input matrix (B), and the desired poles (poles). 

clf                                 %Clear graph
A=[0 1;0 -5.345];                   % Define system matrix A.
B=[0;0.3566];                       % Define input matrix B.
C=[1 0];                            % Define output matrix C.
D=0;                                % Define matrix D.
Tc =input('Type desired time constant:  ');
                                    % Input desired time costant.
z=input('Type desired damping ratio:  ');
                                    % Input desired damping ratio.

                                   
wn=1/(z*Tc);                        % Calculate required natural 
                                    % frequency.
[num,den]=ord2(wn,z);               % Produce a second-order system that 
                                    % meets the transient requirements.
r=roots(den);                       % Use denominator to specify dominant 
                                    % poles.
poles=[r(1) r(2)];                  % Specify pole placement for all 
                                    % poles.
G = acker(A,B,poles)                % Calculate controller gains.
Anew=A-B*G;                         % Form compensated A matrix.
Bnew=[];                             % Form compensated B matrix.
Cnew=C;                             % Form compensated C matrix.
Dnew=D;                             % Form compensated D matrix.
Tss=ss(Anew,Bnew,Cnew,Dnew);        % Form LTI state-space object.
                           

%step(Tss)                           % Produce compensated step response.
x0=[-20;0];
initial(Tss,x0)
title('Compensated Step Response')  % Add title to compensated step 
                                    % response.

