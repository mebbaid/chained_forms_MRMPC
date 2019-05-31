function [z,y] = Prediction_model(x0,u0,Ts,t0)
%Function sample of the dynamical model of the system
% Author: Mohamed Elobaid DIAG Sapienza 2019

z       = zeros(3,1);
z(1)    = x0(1)+Ts*u0(1);
z(2)    = x0(2)+Ts*u0(2);
z(3)    = x0(3)+Ts*x0(2)*u0(1)+Ts^2/2*u0(1)*u0(2);

y       = z;

end

