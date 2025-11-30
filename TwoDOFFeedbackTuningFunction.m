function [q_coeff,p_coeff,alpha_opt] = TwoDOFFeedbackTuningFunction(a,b)
clc;
syms s

[a, b, n] = fb_1determine_n(a, b);
d = fb_2spectral_factorization(a,b);

[L_a,U_a] = fb_3create_LU_matrices(a);
[L_b,U_b] = fb_3create_LU_matrices(b);
[L_d,U_d] = fb_3create_LU_matrices(d);
Sylvester = [L_a, L_b; U_a, U_b];
Right_Matrix = [L_d; U_d];
H = fb_4compute_H_matrix(a,b,d,n);
% 对于一般情况
[rho_H, dom_eig, e, eig_vals] = fb_5compute_eigen_properties(H);
Sylvester = [L_a,L_b;U_a,U_b];
[p_coeff, q_coeff, e_poly,p_poly,q_poly] = fb_6pole_placement(Sylvester, L_d, U_d, e, n);


% 最优控制器
alpha_opt = 1 / sqrt(1 + rho_H^2);

p_coeff = p_coeff';
q_coeff = q_coeff';

end

