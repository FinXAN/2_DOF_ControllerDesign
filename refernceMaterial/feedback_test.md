

determine\_n.m

```matlab
function [a_filled, b_filled, n] = determine_n(a_coeff, b_coeff)
% 自动确定多项式阶数 n 并填充系数向量
% 输入：a_coeff, b_coeff - 多项式系数向量（降幂排列）
% 输出：a_filled, b_filled - 填充后的系数向量，n - 多项式阶数

    % 去除前导零系数
    a_clean = remove_leading_zeros_determine_n(a_coeff);
    b_clean = remove_leading_zeros_determine_n(b_coeff);
    
    % 取两个多项式的最大阶数
    deg_a = length(a_clean) - 1;
    deg_b = length(b_clean) - 1;
    
    n = max(deg_a, deg_b);
    
    % 验证 n >= 1
    if n < 1
        error('多项式阶数必须至少为1');
    end
    
    % 自动填充系数向量到长度 n+1
    a_filled = pad_coefficients_determine_n(a_clean, n);
    b_filled = pad_coefficients_determine_n(b_clean, n);
    
    fprintf('检测到多项式阶数: n = %d\n', n);
    fprintf('a(s) 阶数: %d, b(s) 阶数: %d\n', deg_a, deg_b);
    fprintf('填充后系数: a(s) = ');
    fprintf('%d ', a_filled);
    fprintf(', b(s) = ');
    fprintf('%d ', b_filled);
    fprintf('\n');
end

function clean_coeff = remove_leading_zeros_determine_n(coeff)
% 去除前导零系数
    idx = find(coeff ~= 0, 1);
    if isempty(idx)
        clean_coeff = 0;  % 全零多项式
    else
        clean_coeff = coeff(idx:end);
    end
end

function padded = pad_coefficients_determine_n(coeff, n)
% 用零填充系数向量到长度 n+1
    if length(coeff) < n + 1
        padded = [zeros(1, n + 1 - length(coeff)), coeff];
    else
        padded = coeff(1:n+1);
    end
end

```

spectral\_factorization.m

```matlab
function d_coeff = spectral_factorization(a_coeff, b_coeff)
% 谱分解：求解稳定多项式 d(s) 使得 a(-s)a(s) + b(-s)b(s) = d(-s)d(s)
% 输入：a_coeff, b_coeff - 多项式系数向量（降幂排列）
% 输出：d_coeff - 稳定多项式 d(s) 的系数向量

    % 自动确定多项式阶数
    n_a = get_polynomial_degree(a_coeff);
    n_b = get_polynomial_degree(b_coeff);
    n = max(n_a, n_b);

    fprintf('输入多项式: a(s) 阶数=%d, b(s) 阶数=%d\n', n_a, n_b);
    
    % 步骤1：构造 a(-s)a(s) + b(-s)b(s)
    poly_sum = compute_self_conjugate_sum(a_coeff, b_coeff, n);
    
    fprintf('a(-s)a(s) + b(-s)b(s) 的系数: ');
    fprintf('%.4f ', poly_sum);
    fprintf('\n');
    
    % 步骤2：求根并选择稳定根
    roots_all = roots(poly_sum);
    stable_roots = roots_all(real(roots_all) < 0);
    
    fprintf('所有根: ');
    fprintf('%.4f%+.4fi ', [real(roots_all), imag(roots_all)]');
    fprintf('\n');
    
    fprintf('稳定根: ');
    if ~isempty(stable_roots)
        fprintf('%.4f%+.4fi ', [real(stable_roots), imag(stable_roots)]');
    else
        fprintf('无稳定根');
    end
    fprintf('\n');
    

    if isempty(stable_roots)
        error('未找到稳定根，系统可能不稳定');
    end
    d_coeff = poly(stable_roots);
     % 在得到d_coeff后，添加比例因子校正
    left_side = compute_self_conjugate_sum(a_coeff, b_coeff, n);
    right_side = compute_self_conjugate_sum(d_coeff, zeros(size(d_coeff)), n);
    scale_factor = sqrt(left_side(1) / right_side(1));
    
    d_coeff = scale_factor * d_coeff;

    if d_coeff(1) < 0
        d_coeff = -d_coeff;
    end
    
    fprintf('稳定多项式 d(s) 的系数: ');
    fprintf('%.6f ', d_coeff);
    fprintf('\n');
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
```

compute\_H\_matrix.m

```matlab
function H = compute_H_matrix(a,b,d,n)
    J = create_J_matrix(n);
    [L_a,U_a] = create_LU_matrices(a,n);
    [L_b,U_b] = create_LU_matrices(b,n);
    [L_d,U_d] = create_LU_matrices(d,n);
    
    % 严格按照公式实现
    part1 = J * inv(L_d) * J;
    part2 = [L_b*J, -L_a*J];
    Sylvester = [L_a, L_b; U_a, U_b];
    part3 = [L_d; U_d];
    
    H = part1 * part2 * inv(Sylvester) * part3;
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
```

compute\_eigen\_properties.m

```matlab
function [rho_H, dominant_eigenvalue, e, eigenvalues, m_H] = compute_eigen_properties(H, normalization_type, n)
% 计算矩阵的特征值属性（可选择归一化方式）
% 输入：H - 方阵，normalization_type - 归一化方式（可选），n - 系统阶数
% 输出：同上，增加m_H - 特征值重数

    if nargin < 2
        normalization_type = 'first';
    end
    if nargin < 3
        n = size(H, 1);
    end
    
    % 特征值分解
    [eig_vec, eig_val] = eig(H);
    eigenvalues = diag(eig_val);
    
    % 计算特征值重数m(H)
    abs_eigenvalues = abs(eigenvalues);
    rho_H = max(abs_eigenvalues);
    
    % 找出所有模等于rho_H的特征值索引
    max_indices = find(abs(abs_eigenvalues - rho_H) < 1e-10);
    m_H = length(max_indices);
    
    fprintf('谱半径 ρ(H) = %.6f\n', rho_H);
    fprintf('特征值重数 m(H) = %d\n', m_H);
    
    if m_H > 1
        % 非一般情况：实现Step 3*逻辑
        fprintf('检测到非一般情况，执行Step 3*选择逻辑\n');
        e = select_min_degree_eigenvector(H, eig_vec, max_indices, eigenvalues, rho_H, n);
        dominant_eigenvalue = eigenvalues(find(abs_eigenvalues == rho_H, 1));
    else
        % 一般情况：选择最大特征值对应的特征向量
        [~, idx] = max(abs_eigenvalues);
        e = eig_vec(:, idx);
        dominant_eigenvalue = eigenvalues(idx);
    end
    
    % 输出选择的特征向量信息
    fprintf('选择的特征向量（归一化前）: ');
    for i = 1:length(e)
        if imag(e(i)) == 0
            fprintf('%.6f ', real(e(i)));
        else
            fprintf('%.6f%+.6fi ', real(e(i)), imag(e(i)));
        end
    end
    fprintf('\n');
    
    % 计算并显示e(s)多项式
    e_poly_coeffs = flip(e');
    fprintf('对应的e(s)多项式系数: ');
    fprintf('%.6f ', e_poly_coeffs);
    fprintf('\n');
    
    % 根据选择进行归一化 - 关键修复：智能归一化
    switch normalization_type
        case 'first'
            % 智能选择归一化方式：如果第一个元素接近0，使用单位范数
            if abs(e(1)) > 1e-8
                e_original = e;  % 保存原始向量用于调试
                e = e / e(1);
                fprintf('使第一个元素为1后的特征向量: ');
                fprintf('%.6f ', e);
                fprintf('\n');
            else
                fprintf('特征向量第一个元素接近零，自动使用单位范数归一化\n');
                e = e / norm(e);
                fprintf('单位范数归一化后的特征向量: ');
                fprintf('%.6f ', e);
                fprintf('\n');
            end
            
        case 'unit'
            e = e / norm(e);
            fprintf('单位范数归一化\n');
            
        case 'none'
            fprintf('不进行归一化\n');
            
        otherwise
            error('未知的归一化方式: %s', normalization_type);
    end
end

function e = select_min_degree_eigenvector(H, eig_vec, max_indices, eigenvalues, rho_H, n)
    % 实现Step 3*：选择使e(s)阶数最小的特征向量
    min_degree = n;  % 初始化为最大可能阶数
    best_e = [];
    
    fprintf('候选特征向量分析:\n');
    
    % 首先构建特征空间 E1 (对应ρ(H)) 和 E2 (对应-ρ(H))
    E1 = []; E2 = [];
    for i = 1:length(max_indices)
        idx = max_indices(i);
        lambda = eigenvalues(idx);
        if abs(lambda - rho_H) < 1e-10
            E1 = [E1, eig_vec(:, idx)];
        elseif abs(lambda + rho_H) < 1e-10
            E2 = [E2, eig_vec(:, idx)];
        end
    end
    
    fprintf('  特征空间 E1 (对应ρ(H)=%.6f) 维度: %d\n', rho_H, size(E1, 2));
    fprintf('  特征空间 E2 (对应-ρ(H)=%.6f) 维度: %d\n', -rho_H, size(E2, 2));
    
    % 从 E1 ∪ E2 中选择最小阶数特征向量
    all_candidates = [E1, E2];
    
    if isempty(all_candidates)
        error('在特征空间 E1 ∪ E2 中未找到候选特征向量');
    end
    
    for i = 1:size(all_candidates, 2)
        candidate_e = all_candidates(:, i);
        
        % 计算e(s) = [s^(n-1) s^(n-2) ... 1] * e的实际阶数
        poly_coeffs = flip(candidate_e');  % 转换为多项式系数
        actual_degree = get_actual_polynomial_degree(poly_coeffs);
        
        % 显示候选向量的详细信息
        fprintf('  候选向量 %d: e(s)阶数 = %d\n', i, actual_degree);
        fprintf('    系数向量: ');
        fprintf('%.6f ', poly_coeffs);
        fprintf('\n');
        fprintf('    对应特征值: %.6f\n', eigenvalues(find_corresponding_eigenvalue(candidate_e, eig_vec, eigenvalues)));
        
        if actual_degree < min_degree
            min_degree = actual_degree;
            best_e = candidate_e;
            fprintf('    -> 更新为最佳选择 (当前最小阶数: %d)\n', min_degree);
        end
    end
    
    if isempty(best_e)
        error('未找到合适的特征向量');
    end
    
    fprintf('最终选择: e(s)阶数 = %d\n', min_degree);
    
    e = best_e;
end

function degree = get_actual_polynomial_degree(coeffs)
    % 计算多项式的实际阶数（忽略前导零）
    idx = find(abs(coeffs) > 1e-10, 1);
    if isempty(idx)
        degree = 0;  % 零多项式
    else
        degree = length(coeffs) - idx;
    end
end

function lambda_idx = find_corresponding_eigenvalue(v, eig_vec, eigenvalues)
    % 找到特征向量v对应的特征值索引
    for i = 1:size(eig_vec, 2)
        if norm(eig_vec(:, i) - v) < 1e-10 || norm(eig_vec(:, i) + v) < 1e-10
            lambda_idx = i;
            return;
        end
    end
    lambda_idx = 1;  % 默认返回第一个（理论上不应该发生）
end

```

pole\_placement.m

```matlab
function [p_coeff, q_coeff, e_poly,p_poly,q_poly] = pole_placement(Sylvester, L_d, U_d, e, n)
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
    
    for i = 1:length(e)
        if imag(e(i)) == 0
            fprintf('  %.6f\n', real(e(i)));
        else
            fprintf('  %.6f%+.6fi\n', real(e(i)), imag(e(i)));
        end
    end
    
    
    fprintf('最优控制器: C_opt(s) = q(s)/p(s)\n');
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
```

TwoDOFFeedbackTuningFunction.m

```matlab
function [q_coeff,p_coeff,alpha_opt] = TwoDOFFeedbackTuningFunction(a,b)
    clc;
    syms s
    [a, b, n] = determine_n(a, b);
    d = spectral_factorization(a,b);
    [L_a,U_a] = create_LU_matrices(a);
    [L_b,U_b] = create_LU_matrices(b);
    [L_d,U_d] = create_LU_matrices(d);
    Sylvester = [L_a, L_b; U_a, U_b];
    H = compute_H_matrix(a,b,d,n);
    
    fprintf('计算得到的H矩阵:\n');
    disp(H);
    
    % 使用单位范数归一化，避免数值问题
    [rho_H, dom_eig, e, eig_vals, m_H] = compute_eigen_properties(H, 'unit', n);
    
    [p_coeff, q_coeff, e_poly,p_poly,q_poly] = pole_placement(Sylvester, L_d, U_d, e, n);
    % 最优控制器
    alpha_opt = 1 / sqrt(1 + rho_H^2);
    p_coeff = p_coeff';
    q_coeff = q_coeff';
end
```

函数调用

```matlab
a = [1, -6, 11,-6]; 
b = [1,6,11,6];    

[num,denum,alpha_opt]=TwoDOFFeedbackTuningFunction(a,b)
```

```matlabTextOutput
检测到多项式阶数: n = 3
a(s) 阶数: 3, b(s) 阶数: 3
填充后系数: a(s) = 1 -6 11 -6 , b(s) = 1 6 11 6 
输入多项式: a(s) 阶数=3, b(s) 阶数=3
a(-s)a(s) + b(-s)b(s) 的系数: -2.0000 0.0000 28.0000 0.0000 -98.0000 0.0000 72.0000 
所有根: -3.0000+0.0000i 3.0000+0.0000i -2.0000+0.0000i -1.0000+0.0000i 2.0000+0.0000i 1.0000+0.0000i 
稳定根: -3.0000+0.0000i -2.0000+0.0000i -1.0000+0.0000i 
稳定多项式 d(s) 的系数: 1.414214 8.485281 15.556349 8.485281 
谱分解无明显误差
计算得到的H矩阵:
   -1.0000   -0.0000    0.0000
    0.0000    1.0000    0.0000
    0.0000   -0.0000   -1.0000

谱半径 ρ(H) = 1.000000
特征值重数 m(H) = 3
检测到非一般情况，执行Step 3*选择逻辑
候选特征向量分析:
  特征空间 E1 (对应ρ(H)=1.000000) 维度: 1
  特征空间 E2 (对应-ρ(H)=-1.000000) 维度: 2
  候选向量 1: e(s)阶数 = 1
    系数向量: 0.000000 -1.000000 0.000000 
    对应特征值: 1.000000
    -> 更新为最佳选择 (当前最小阶数: 1)
  候选向量 2: e(s)阶数 = 2
    系数向量: -0.154034 -0.000000 0.988066 
    对应特征值: -1.000000
  候选向量 3: e(s)阶数 = 2
    系数向量: -0.999972 0.000000 -0.007450 
    对应特征值: -1.000000
最终选择: e(s)阶数 = 1
选择的特征向量（归一化前）: 0.000000 -1.000000 0.000000 
对应的e(s)多项式系数: 0.000000 -1.000000 0.000000 
单位范数归一化
  0.000000
  -1.000000
  0.000000
最优控制器: C_opt(s) = q(s)/p(s)
num = 1x3
   -0.0000   -1.4142    0.0000

denum = 1x3
1.0e-15 *

    0.3465    0.5581    0.4830

alpha_opt = 0.7071
```


