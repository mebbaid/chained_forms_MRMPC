function z = ChainedForm(x, u, Ts)
delta = 0.1;
deltabar = delta/2;

z = zeros(3,1);
z(1) = x(1) + delta*u(1);
z(2) = x(2) + deltabar*(u(2)+u(3));
z(3) = x(3) + 2*deltabar*u(1)*x(2) +  (3*deltabar^2)/2*u(1)*u(2) + (deltabar^2)/2*u(1)*u(3);
end

