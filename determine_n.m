function [a_filled, b_filled, n] = determine_n(a_coeff, b_coeff)
% 自动确定多项式阶数 n 并填充系数向量
% 输入：a_coeff, b_coeff - 多项式系数向量（降幂排列）
% 输出：a_filled, b_filled - 填充后的系数向量，n - 多项式阶数

    % 去除前导零系数
    a_clean = remove_leading_zeros(a_coeff);
    b_clean = remove_leading_zeros(b_coeff);
    
    % 取两个多项式的最大阶数
    deg_a = length(a_clean) - 1;
    deg_b = length(b_clean) - 1;
    
    n = max(deg_a, deg_b);
    
    % 验证 n >= 1
    if n < 1
        error('多项式阶数必须至少为1');
    end
    
    % 自动填充系数向量到长度 n+1
    a_filled = pad_coefficients(a_clean, n);
    b_filled = pad_coefficients(b_clean, n);
    
    fprintf('检测到多项式阶数: n = %d\n', n);
    fprintf('a(s) 阶数: %d, b(s) 阶数: %d\n', deg_a, deg_b);
    fprintf('填充后系数: a(s) = ');
    fprintf('%d ', a_filled);
    fprintf(', b(s) = ');
    fprintf('%d ', b_filled);
    fprintf('\n');
end

function clean_coeff = remove_leading_zeros(coeff)
% 去除前导零系数
    idx = find(coeff ~= 0, 1);
    if isempty(idx)
        clean_coeff = 0;  % 全零多项式
    else
        clean_coeff = coeff(idx:end);
    end
end

function padded = pad_coefficients(coeff, n)
% 用零填充系数向量到长度 n+1
    if length(coeff) < n + 1
        padded = [zeros(1, n + 1 - length(coeff)), coeff];
    else
        padded = coeff(1:n+1);
    end
end
