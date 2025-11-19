function C_opt = compute_optimal_controller(a_coeff, b_coeff)
% 计算最优鲁棒控制器
    
    % Step 1: 确定阶数 n
    [a_filled, b_filled, n] = determine_n(a_coeff, b_coeff);
    
    % Step 2: 谱分解
    d_coeff = spectral_factorization(a_filled, b_filled);
    
    % Step 3: 构造矩阵
    [H, Sylvester, L_d, U_d] = construct_matrices(a_filled, b_filled, d_coeff, n);
    
    % Step 4: 特征值计算
    [rho_H, dominant_eigenvalue, e, eigenvalues] = compute_eigen_properties(H);
    
    % Step 5: 极点配置
    [p_coeff, q_coeff, e_poly] = pole_placement(Sylvester, L_d, U_d, e, n);
    
    % Step 6: 计算最优鲁棒性裕度
    alpha = 1 / sqrt(1 + rho_H^2);
    
    fprintf('\n=== 最终结果 ===\n');
    fprintf('最优控制器: C_opt(s) = q(s)/p(s)\n');
    fprintf('最优鲁棒性裕度: α = %.6f\n', alpha);
    fprintf('谱半径: ρ(H) = %.6f\n', rho_H);
    
    % 返回控制器
    C_opt = @(s) polyval(q_coeff', s) ./ polyval(p_coeff', s);
end
