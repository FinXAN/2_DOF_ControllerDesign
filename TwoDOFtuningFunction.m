function [Q1,C1,C2] = TwoDOFtuningFunction(num,denum,Q2,alphas)

arguments
    num (1,:){mustBeNumeric}
    denum (1,:){mustBeNumeric}
    Q2 {mustBeA(Q2, 'tf')}
    alphas (1,:){mustBeNumeric} = [3,3,3,3];
end

b = num;
a = denum;
P = tf(b,a);

C0 = tf(pidtune(P, "PIDF"));
q = C0.num{1};
p = C0.den{1};
% c(s) = a(s)p(s) + b(s)q(s)
cpoly1 = conv(a, p);
cpoly2 = conv(b, q);

% 确保数组长度相同
max_len = max(length(cpoly1), length(cpoly2));
cpoly1_padded = [zeros(1, max_len - length(cpoly1)), cpoly1];
cpoly2_padded = [zeros(1, max_len - length(cpoly2)), cpoly2];

% 现在可以相加
cpoly = cpoly1_padded + cpoly2_padded;
r = roots(cpoly);

n = numel(a) - 1;       
m = numel(p) - 1;
[f, h] = fw_1split_roots_real_coeffs(r, n);
% 验证：deg f = n, deg h = m
assert(length(f)-1 == n, 'deg f != n');
assert(length(h)-1 == m, 'deg h != m');
M = tf(a,f);
N = tf(b,f);
X = tf(p,h);
Y = tf(q,h);[f, h] = fw_1split_roots_real_coeffs(r, n);
% 验证
assert(length(f)-1 == n, 'deg f != n');
assert(length(h)-1 == m, 'deg h != m');
 

b_roots = roots(b);
stable_roots = b_roots(real(b_roots) < 0);  
unstable_roots = b_roots(real(b_roots) >= 0); 

% 构建b_s(s)和b_u(s)
b_s = poly(stable_roots);    
b_u = poly(unstable_roots);  
b_u_0 = polyval(b_u, 0);
% ========== 新增：计算k值 ==========
deg_f = length(f) - 1;           % deg f(s)
deg_b_s = length(b_s) - 1;       % deg b_s(s)
min_k = deg_f - deg_b_s;
current_k = length(alphas);
if current_k < min_k
    warning('k值不足: 当前k=%d, 需要k≥%d。自动调整k值。', current_k, min_k);
    % 补充额外的α值
    additional_alphas = 5 * ones(1, min_k - current_k);  % 使用较小的α值
    alphas = [alphas, additional_alphas];
    fprintf('调整后的alphas: %s\n', mat2str(alphas));
end

b_u_0_value = polyval(b_u, 0);  % 这是标量值
denominator_poly = b_s * b_u_0_value;  % b_s(s) * b_u(0)
Q1_base = tf(f, denominator_poly);

% 然后乘以抵消项
for a_i = alphas
    Q1_base = Q1_base * tf(a_i, [1 a_i]);
end
Q1 = minreal(Q1_base);
C1 = minreal(Q1/(X-N*Q2));
C2  = minreal((Y+M*Q2)/(X-N*Q2));
end

function [f, h] = fw_1split_roots_real_coeffs(roots, n)
    % SPLIT_ROOTS_REAL_COEFFS 将根分配给两个实系数多项式
    % 输入:
    %   roots - 所有根的向量
    %   n     - f(s)的度数
    % 输出:
    %   f - 度数为n的实系数多项式系数
    %   h - 剩余根构成的实系数多项式系数
    
    tol = 1e-10;
    
    % 分离实根和复根
    is_real = abs(imag(roots)) < tol;
    real_roots = roots(is_real);
    complex_roots = roots(~is_real);
    
    % 预分配存储空间
    max_pairs = floor(length(complex_roots) / 2);
    complex_pairs = zeros(max_pairs, 2);
    pair_count = 0;
    used = false(size(complex_roots));
    
    % 高效配对复根
    for i = 1:length(complex_roots)
        if used(i), continue; end
        
        root_i = complex_roots(i);
        conj_root = conj(root_i);
        
        % 向量化查找共轭根
        dist = abs(complex_roots - conj_root);
        [min_dist, conj_idx] = min(dist);
        
        if min_dist < tol && ~used(conj_idx)
            pair_count = pair_count + 1;
            complex_pairs(pair_count, :) = [root_i, complex_roots(conj_idx)];
            used([i, conj_idx]) = true;
        end
    end
    
    % 裁剪到实际大小
    complex_pairs = complex_pairs(1:pair_count, :);
    
    % 分配根给f(s)
    num_real_f = min(length(real_roots), n);
    remaining_slots = n - num_real_f;
    num_pairs_f = min(floor(remaining_slots / 2), pair_count);
    
    % 检查度数约束
    total_roots_f = num_real_f + 2 * num_pairs_f;
    if total_roots_f ~= n
        error('无法构造度数为%d的实系数多项式，可用根数不匹配', n);
    end
    
    % 构建根向量
    roots_f = [real_roots(1:num_real_f); 
               reshape(complex_pairs(1:num_pairs_f, :)', [], 1)];
    
    roots_h = [real_roots(num_real_f+1:end);
               reshape(complex_pairs(num_pairs_f+1:end, :)', [], 1)];
    
    % 构建实系数多项式
    f = real(poly(roots_f));
    h = real(poly(roots_h));
    
    % 验证
    assert(length(f) - 1 == n, 'f的度数不正确');
    assert(all(abs(imag(f)) < tol), 'f有复数系数');
    assert(all(abs(imag(h)) < tol), 'h有复数系数');
end
