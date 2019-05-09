%% This is a general MR planner script 
% Author Mohamed Elobaid , Sapienza 2019

function F_mr = MR_Planner(F_delta, z, u, mr, ctr)



z = sym('z', [length(F_delta),1]); assume(z, 'real');
u = sym('u', [length(u),1]); assume(u, 'real');

ref = sym('ref', [length(F_delta),1]); assume(ref, 'real'); 
delta_b = sym('delta_b', 'real');

z_1 = z(1:2);
z_2 = z(3:6);

F_2 = z_2;
F_1 = z_1; 
f2_tmp = F_2;
f1_tmp = F_1; 

for i = 1:3
    
    j2 = jacobian(f2_tmp, z_2)*(f2);
    f2_tmp = j2;
    F_2 = simplify(F_2 + delta_b^i/factorial(i)*f2_tmp);
    j1 = jacobian(f1_tmp, z_1)*(f1);
    f1_tmp = j1;
    F_1 = simplify(F_1 + delta_b^i/factorial(i)*f1_tmp);
end

%F_2 is hence the SR SD equivalent model%


%Compute the MR SD equivalent model%

u11 = sym('u11', 'real'); 
u12 = sym('u12', 'real'); 

u21 = sym('u21', 'real'); 
u22 = sym('u22', 'real'); 
u23 = sym('u23', 'real'); 
u24 = sym('u24', 'real'); 

u_1 = [u11; u21]; 
u_2 = [u11; u22]; 
u_3 = [u12; u23]; 
u_4 = [u12; u24];

u_mr = [u_1 u_2 u_3 u_4]; 

F1_MR = z_1; 
F2_MR = z_2; 

for i = 1:4
    F1_MR = simplify(subs(F_1, [z_1, u], [F1_MR, u_mr(:, i)]));
    F2_MR = simplify(subs(F_2, [z_2, u], [F2_MR, u_mr(:, i)])); 
end

A1 = jacobian(F1_MR, z_1); 
A2 = jacobian(F2_MR, z_2); 

B1 = jacobian(F1_MR, [u11; u12]); 

B21 = jacobian(F2_MR, [u21; u22; u23; u24]);

feed1 = simplify(inv(B1)*(ref(1:2)-A1*z(1:2))); 
B21 = simplify(subs(B21, [u11; u12], feed1)); 
feed2 = simplify(inv(B21)*(ref(3:6)-A2*z(3:6)));

end 
