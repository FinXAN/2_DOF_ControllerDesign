function d_coeff = spectral_factorization(a_coeff, b_coeff)
% 谱分解：求解稳定多项式 d(s) 使得 a(-s)a(s) + b(-s)b(s) = d(-s)d(s)
% 输入：a_coeff, b_coeff - 多项式系数向量（降幂排列）
% 输出：d_coeff - 稳定多项式 d(s) 的系数向量

    % 自动确定多项式阶数
    n_a = get_polynomial_degree(a_coeff);
    n_b = get_polynomial_degree(b_coeff);
    n = max(n_a, n_b);

    %fprintf('输入多项式: a(s) 阶数=%d, b(s) 阶数=%d\n', n_a, n_b);
    
    % 步骤1：构造 a(-s)a(s) + b(-s)b(s)
    poly_sum = compute_self_conjugate_sum(a_coeff, b_coeff, n);
    
    %fprintf('a(-s)a(s) + b(-s)b(s) 的系数: ');
    %fprintf('%.4f ', poly_sum);
    %fprintf('\n');
    
    % 步骤2：求根并选择稳定根
    roots_all = roots(poly_sum);
    stable_roots = roots_all(real(roots_all) < 0);
    
    %fprintf('所有根: ');
    %fprintf('%.4f%+.4fi ', [real(roots_all), imag(roots_all)]');
    %fprintf('\n');
    
    %fprintf('稳定根: ');
    %if ~isempty(stable_roots)
    %    fprintf('%.4f%+.4fi ', [real(stable_roots), imag(stable_roots)]');
    %else
    %    fprintf('无稳定根');
    %end
    %fprintf('\n');
    

    if isempty(stable_roots)
        error('未找到稳定根，系统可能不稳定');
    end
    
    d_coeff = poly(stable_roots);
    

    if d_coeff(1) < 0
        d_coeff = -d_coeff;
    end
    
    %fprintf('稳定多项式 d(s) 的系数: ');
    %fprintf('%.6f ', d_coeff);
    %fprintf('\n');
    verify_factorization(a_coeff, b_coeff, d_coeff, n);

end

function degree = get_polynomial_degree(coeff)
    idx = find(coeff ~= 0, 1);
    if isempty(idx)
        degree = 0;
    else
        degree = length(coeff) - idx;
    end
end

function poly_sum = compute_self_conjugate_sum(a_coeff, b_coeff, n)
% 计算 a(-s)a(s) + b(-s)b(s)
    
    % 确保系数向量长度一致
    a_padded = pad_coefficients(a_coeff, n);
    b_padded = pad_coefficients(b_coeff, n);
    
    % 计算 a(-s)a(s)
    a_minus_s = compute_polynomial_at_minus_s(a_padded);
    a_product = conv(a_minus_s, a_padded);
    
    % 计算 b(-s)b(s)  
    b_minus_s = compute_polynomial_at_minus_s(b_padded);
    b_product = conv(b_minus_s, b_padded);
    
    % 求和
    max_len = max(length(a_product), length(b_product));
    a_product_padded = [zeros(1, max_len - length(a_product)), a_product];
    b_product_padded = [zeros(1, max_len - length(b_product)), b_product];
    
    poly_sum = a_product_padded + b_product_padded;
    
    % 去除前导零
    poly_sum = remove_leading_zeros(poly_sum);
end

function coeff_minus_s = compute_polynomial_at_minus_s(coeff)
% 计算多项式在 -s 处的系数：f(-s)
    n = length(coeff) - 1;
    signs = (-1).^(n:-1:0);
    coeff_minus_s = coeff .* signs;
end

function padded = pad_coefficients(coeff, n)
% 用零填充系数向量到长度 n+1
    if length(coeff) < n + 1
        padded = [zeros(1, n + 1 - length(coeff)), coeff];
    else
        padded = coeff(1:n+1);
    end
end

function clean = remove_leading_zeros(coeff)
% 去除前导零
    idx = find(coeff ~= 0, 1);
    if isempty(idx)
        clean = 0;
    else
        clean = coeff(idx:end);
    end
end

function verify_factorization(a_coeff, b_coeff, d_coeff, n)
% 验证分解结果 
    % 计算左边：a(-s)a(s) + b(-s)b(s)
    left_side = compute_self_conjugate_sum(a_coeff, b_coeff, n);
    
    % 计算右边：d(-s)d(s)
    d_padded = pad_coefficients(d_coeff, n);
    d_minus_s = compute_polynomial_at_minus_s(d_padded);
    right_side = conv(d_minus_s, d_padded);
    right_side = remove_leading_zeros(right_side);
    
    % 比较误差
    max_len = max(length(left_side), length(right_side));
    left_padded = [zeros(1, max_len - length(left_side)), left_side];
    right_padded = [zeros(1, max_len - length(right_side)), right_side];
    
    error = norm(left_padded - right_padded);

    
    if error < 1e-10
        fprintf('谱分解无明显误差\n');
    else
        fprintf('分解误差较大，请检查计算\n');
    end
end
