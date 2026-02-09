function C1 = TwoDOFtuningFunction(num,denum,C2,alphas)

arguments
    num (1,:){mustBeNumeric}
    denum (1,:){mustBeNumeric}
    C2 {mustBeA(C2, 'tf')}
    alphas (1,:){mustBeNumeric} = [3,3,3,3];
end

b = num;
a = denum;
P = tf(b,a);

C0 = C2;
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
[f, h] = fw_1split_roots_real_coeffs(r, n,m);
% 验证：deg f = n, deg h = m
assert(length(f)-1 == n, 'deg f != n');
assert(length(h)-1 == m, 'deg h != m');
M = tf(a,f);
N = tf(b,f);
X = tf(p,h);
Y = tf(q,h);[f, h] = fw_1split_roots_real_coeffs(r, n,m);
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
C1 = minreal(Q1/(X)); % 因为 Q2 是0，C1 按照参数化就是直接设计出来的   
% 9.normlization for elimiatnating pole at (0,0)
[~, den1] = tfdata(C1, 'v');
if den1(end) == 0
    s = tf('s');
    C1 = minreal(C1*s); % remove exactly one integrator;
end
G0 = minreal((P*C1)/(1 + P*C2));
K  = 1/dcgain(G0);
C1 = K*C1;
end

function [f, h] = fw_1split_roots_real_coeffs(r, n, m)
    % 改进的根分解函数
    % 输入：
    %   r - 闭环特征多项式的所有根
    %   n - f(s)的期望阶数（被控对象分母阶数）
    %   m - h(s)的期望阶数（控制器分母阶数）
    % 输出：
    %   f, h - 实系数多项式系数向量
    
    tol = 1e-10;
    
    % 验证总根数
    total_roots = length(r);
    if total_roots ~= (n + m)
        error('根数不匹配: 总根数=%d, 但n+m=%d', total_roots, n+m);
    end
    
    % 1. 分离实根和复根
    is_real = abs(imag(r)) < tol;
    real_roots = r(is_real);
    complex_roots = r(~is_real);
    
    % 2. 确保复根成对出现
    if mod(length(complex_roots), 2) ~= 0
        error('复根不成对出现，无法构造实系数多项式');
    end
    
    % 3. 按实部排序（最稳定的根给f(s)）
    [~, idx] = sort(real(r));
    r_sorted = r(idx);
    
    % 4. 简单分配：前n个根给f(s)，剩余给h(s)
    roots_f = r_sorted(1:n);
    roots_h = r_sorted(n+1:end);
    
    % 5. 验证分配后的复根配对
    % 检查f(s)的复根是否成对
    complex_in_f = roots_f(abs(imag(roots_f)) > tol);
    if ~isempty(complex_in_f)
        for i = 1:length(complex_in_f)
            root = complex_in_f(i);
            conj_root = conj(root);
            if ~any(abs(roots_f - conj_root) < tol)
                warning('f(s)中的复根 %s 没有共轭配对', num2str(root));
            end
        end
    end
    
    % 检查h(s)的复根是否成对
    complex_in_h = roots_h(abs(imag(roots_h)) > tol);
    if ~isempty(complex_in_h)
        for i = 1:length(complex_in_h)
            root = complex_in_h(i);
            conj_root = conj(root);
            if ~any(abs(roots_h - conj_root) < tol)
                warning('h(s)中的复根 %s 没有共轭配对', num2str(root));
            end
        end
    end
    
    % 6. 构建多项式
    f = real(poly(roots_f));
    h = real(poly(roots_h));
    
    % 7. 验证阶数
    if length(f) - 1 ~= n
        error('f(s)阶数错误: 期望%d, 实际%d', n, length(f)-1);
    end
    if length(h) - 1 ~= m
        error('h(s)阶数错误: 期望%d, 实际%d', m, length(h)-1);
    end
    
    % 8. 验证系数为实数
    if any(abs(imag(f)) > tol)
        error('f(s)包含复数系数');
    end
    if any(abs(imag(h)) > tol)
        error('h(s)包含复数系数');
    end

end

