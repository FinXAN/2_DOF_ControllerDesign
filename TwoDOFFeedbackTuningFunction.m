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

function [a_filled, b_filled, n] = fb_1determine_n(a_coeff, b_coeff)
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
    
    %fprintf('检测到多项式阶数: n = %d\n', n);
    %fprintf('a(s) 阶数: %d, b(s) 阶数: %d\n', deg_a, deg_b);
    %fprintf('填充后系数: a(s) = ');
    %fprintf('%d ', a_filled);
    %fprintf(', b(s) = ');
    %fprintf('%d ', b_filled);
    %fprintf('\n');
end


function d_coeff = fb_2spectral_factorization(a_coeff, b_coeff)
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


function [L, U] = fb_3create_LU_matrices(f_coeff, n)
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

function H = fb_4compute_H_matrix(a,b,d,n)
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


function [rho_H, dominant_eigenvalue, e, eigenvalues] = fb_5compute_eigen_properties(H, normalization_type)
% 计算矩阵的特征值属性（可选择归一化方式）
% 输入：H - 方阵，normalization_type - 归一化方式（可选）
%        'first' - 使第一个元素为1（默认）
%        'unit' - 单位范数
%        'none' - 不归一化
% 输出：同上

    if nargin < 2
        normalization_type = 'first';
    end
    
    
    % 特征值分解
    [eig_vec, eig_val] = eig(H);
    eigenvalues = diag(eig_val);
    
    % 找到谱半径 ρ(H)
    [rho_H, idx] = max(abs(eigenvalues));
    dominant_eigenvalue = eigenvalues(idx);
    e = eig_vec(:, idx);
    
    % 根据选择进行归一化
    switch normalization_type
        case 'first'
            if abs(e(1)) > eps
                e = e / e(1);
                %fprintf('使第一个元素为1\n');
            else
                warning('特征向量第一个元素接近零，使用单位归一化');
                e = e / norm(e);
            end
            
        case 'unit'
            e = e / norm(e);
            fprintf('unit\n');
            
        case 'none'
            fprintf('none\n');
            
        otherwise
            error('未知的归一化: %s', normalization_type);
    end
    
    
    
end

function [p_coeff, q_coeff, e_poly,p_poly,q_poly] = fb_6pole_placement(Sylvester, L_d, U_d, e, n)
% 极点配置计算 p(s) 和 q(s) 的系数
% 输入：Sylvester - Sylvester矩阵，L_d, U_d - d(s)的矩阵，e - 特征向量，n - 阶数
% 输出：p_coeff, q_coeff - 多项式系数，e_poly - e(s)多项式
    
    % 验证输入维度
    if size(Sylvester,1) ~= 2*n || size(Sylvester,2) ~= 2*n
        error('Sylvester矩阵维度应为 %dx%d', 2*n, 2*n);
    end
    
    if length(e) ~= n
        error('特征向量维度应为 %d', n);
    end
    
    % 根据论文公式(10): [p; q] = Sylvester^{-1} * [L_d; U_d] * e
    pq_vector = Sylvester \ [L_d; U_d] * e;
    
    % 提取 p 和 q 的系数
    p_coeff = pq_vector(1:n);
    q_coeff = pq_vector(n+1:end);
    
    % 构造 e(s) 多项式
    e_poly = flip(e');  % 从特征向量构造多项式系数
    
    %for i = 1:length(e)
    %    if imag(e(i)) == 0
    %        fprintf('  %.6f\n', real(e(i)));
    %    else
    %        fprintf('  %.6f%+.6fi\n', real(e(i)), imag(e(i)));
    %    end
    %end
    
    
    %fprintf('最优控制器: C_opt(s) = q(s)/p(s)\n');
    p_poly = coeffs_to_polynomial(p_coeff);
    q_poly = coeffs_to_polynomial(q_coeff);

end

function print_polynomial(coeff, var)
% 打印多项式
    terms = {};
    n = length(coeff) - 1;
    
    for i = 1:length(coeff)
        power = n - i + 1;
        if abs(coeff(i)) > 1e-10
            if power == 0
                terms{end+1} = sprintf('%.6f', coeff(i));
            elseif power == 1
                if abs(coeff(i) - 1) < 1e-10
                    terms{end+1} = sprintf('%s', var);
                else
                    terms{end+1} = sprintf('%.6f%s', coeff(i), var);
                end
            else
                if abs(coeff(i) - 1) < 1e-10
                    terms{end+1} = sprintf('%s^%d', var, power);
                else
                    terms{end+1} = sprintf('%.6f%s^%d', coeff(i), var, power);
                end
            end
        end
    end
    
    if isempty(terms)
        fprintf('0\n');
    else
        fprintf('%s', terms{1});
        for i = 2:length(terms)
            if coeff(i) > 0
                fprintf(' + %s', terms{i});
            else
                fprintf(' - %s', terms{i}(2:end));
            end
        end
        fprintf('\n');
    end

    
end
function poly_s = coeffs_to_polynomial(coeffs, var_name)
% 将系数向量转换为符号多项式
% 输入：coeffs - 多项式系数（从高阶到低阶），var_name - 变量名（默认为's'）
% 输出：poly_s - 符号多项式

    if nargin < 2
        var_name = 's';
    end
    
    syms(var_name);
    s = eval(var_name);
    
    n = length(coeffs) - 1;  % 多项式阶数
    poly_s = 0;
    
    for i = 1:length(coeffs)
        power = n - i + 1;
        coeff = coeffs(i);
        
        if abs(coeff) > 1e-10  % 忽略接近零的系数
            if power == 0
                term = coeff;
            elseif power == 1
                term = coeff * s;
            else
                term = coeff * s^power;
            end
            
            poly_s = poly_s + term;
        end
    end
end
