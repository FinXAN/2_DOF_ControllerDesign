function H = compute_H_matrix(a,b,d,n)
    J = create_J_matrix(n);
    [L_a,U_a] = create_LU_matrices(a,n);
    [L_b,U_b] = create_LU_matrices(b,n);
    [L_d,U_d] = create_LU_matrices(d,n);
    
    first = J * (L_d \ J);
    second = [L_b * J, -L_a * J];
    third = [L_a, L_b; U_a, U_b];
    fourth = [L_d; U_d];
    
    % 关键修正：明确括号
    H = first * second * (third \ fourth);
end

function [L, U] = create_LU_matrices(f_coeff, n)
% 创建L和U矩阵，支持手动选择矩阵维度
% f_coeff: 多项式系数向量 [f0, f1, ..., f_m]
% n: 可选参数，指定矩阵的维度 (n×n)
%     如果不提供n，则默认n = length(f_coeff) - 1
% 处理可选参数
if nargin < 2
    n = length(f_coeff) - 1;  % 默认使用多项式阶数
end
% 确保n是正整数
n = max(1, round(n));
% 扩展或截断系数向量以匹配维度n
if length(f_coeff) < n+1
    % 如果系数不够，用0填充
    f_coeff_padded = [f_coeff, zeros(1, n+1 - length(f_coeff))];
else
    % 如果系数过多，截断到n+1个
    f_coeff_padded = f_coeff(1:n+1);
end
% L矩阵：使用f0到f_{n-1}作为第一列
L_col = f_coeff_padded(1:n);
L = toeplitz(L_col, [L_col(1), zeros(1, n-1)]);
% U矩阵：使用f_n到f_1作为第一行
U_row = fliplr(f_coeff_padded(2:end));
U = toeplitz([U_row(1), zeros(1, n-1)], U_row);
end

function J = create_J_matrix(n)
% 创建J矩阵（符号矩阵）
% n: 矩阵维度
% J: n×n对角矩阵，对角线元素为 [(-1)^(n-1), (-1)^(n-2), ..., -1, 1]
% 生成对角线元素
diag_elements = (-1).^((n-1):-1:0);
% 创建对角矩阵
J = diag(diag_elements);
end