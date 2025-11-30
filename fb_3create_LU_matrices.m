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