function [L_matrix, U_matrix] = generate_LU_matrices(poly_coeff, system_order)
% 从多项式系数自动生成L和U矩阵
%
% 输入参数:
%   poly_coeff  - 多项式系数向量 [c₀, c₁, ..., cₙ] (从高次到低次)
%   system_order - 系统阶数 n
%
% 输出参数:
%   L_matrix - L矩阵 (n×n)
%   U_matrix - U矩阵 (n×n)
%
% 示例:
%   [L_a, U_a] = generate_LU_matrices([1, 0, 0], 2)
%   [L_b, U_b] = generate_LU_matrices([0, 0, 1], 2) 
%   [L_d, U_d] = generate_LU_matrices([1, sqrt(2), 1], 2)
    % 参数验证
    if length(poly_coeff) ~= system_order + 1
        error('多项式系数长度应为 n+1，当前长度 %d，期望 %d', ...
              length(poly_coeff), system_order + 1);
    end
    
    n = system_order;
    
    % 初始化矩阵
    L_matrix = zeros(n);
    U_matrix = zeros(n);
    
    % 构造L矩阵 (低次部分)
    % L(i,j) = poly_coeff(n - (j-i) + 1)
    for i = 1:n
        for j = i:n
            index = n - (j - i) + 1;
            if index <= length(poly_coeff) && index >= 1
                L_matrix(i, j) = poly_coeff(index);
            end
        end
    end
    
    % 构造U矩阵 (高次部分)
    % U(i,j) = poly_coeff(i + j - 1)
    for i = 1:n
        for j = 1:(n - i + 1)
            index = i + j - 1;
            if index <= length(poly_coeff) && index >= 1
                U_matrix(i, j) = poly_coeff(index);
            end
        end
    end
end