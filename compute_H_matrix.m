function H_matrix = compute_H_matrix(a_coeff, b_coeff, d_coeff, system_order)
% 计算H矩阵的完整函数（修正版）
%
% 输入参数:
%   a_coeff, b_coeff, d_coeff - 多项式系数向量
%   system_order - 系统阶数 n
%
% 输出参数:
%   H_matrix - 计算得到的H矩阵

    n = system_order;
    
    % 生成所有需要的矩阵
    [L_a, U_a] = generate_LU_matrices(a_coeff, n);
    [L_b, U_b] = generate_LU_matrices(b_coeff, n);
    [L_d, U_d] = generate_LU_matrices(d_coeff, n);
    
    % 交换矩阵
    J = flip(eye(n));
    
    % Sylvester矩阵及其逆
    Sylvester = [L_a, L_b; U_a, U_b];
    Sylvester_inv = inv(Sylvester);
    
    % 右侧矩阵
    Right_Matrix = [L_d; U_d];
    
    % 修正：正确的矩阵乘法顺序
    % H = J·inv(L_d)·J·[L_b·J - L_a·J]·inv(S)·[L_d; U_d]
    
    % 分步计算（确保维度匹配）
    term1 = J * inv(L_d) * J;                    % n×n
    term2 = (L_b * J) - (L_a * J);               % n×n  
    term3 = Sylvester_inv * Right_Matrix;        % 2n×n
    
    % 关键修正：需要提取term3的上半部分
    term3_upper = term3(1:n, :);                 % n×n
    
    H_matrix = term1 * term2 * term3_upper;      % 现在维度匹配了！
end
