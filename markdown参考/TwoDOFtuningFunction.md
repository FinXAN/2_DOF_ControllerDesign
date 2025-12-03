function [C1, C2] = TwoDOFtuningFunction(num, den, Q2, alphas)

​    arguments

​        num (1,:){mustBeNumeric}

​        den (1,:){mustBeNumeric}

​        Q2 {mustBeA(Q2, 'tf')}

​        alphas (1,:){mustBeNumeric} = [3,3,3,3];

​    end

​    

​    b = num;

​    a = den;

​    P = tf(b, a);

​    

​    % 1. 设计初始控制器（使用更合适的方法）

​    % 使用极点配置而不是PID

​    n = length(a) - 1;

​    

​    % 选择期望的闭环极点（比被控对象快一些）

​    desired_poles = -linspace(1, n+1, n) * 2;  % 简单的极点选择

​    

​    % 使用极点配置设计控制器

​    [K, ~, ~] = place(ss(P), desired_poles);

​    C0 = tf(K);

​    

​    % 或者使用更简单的方法：直接设计一个稳定的控制器

​    if n == 1

​        % 一阶系统

​        p = [1, 5];  % 控制器分母

​        q = [10];    % 控制器分子

​    elseif n == 2

​        % 二阶系统

​        p = [1, 3, 2];  % s^2 + 3s + 2

​        q = [5, 1];     % 5s + 1

​    else

​        % 高阶系统，使用简单设计

​        p = [1, ones(1, n-1)*2];  % 稳定的分母

​        q = [ones(1, n-1)*3, 1];  % 分子

​    end

​    

​    % 2. 计算闭环特征多项式

​    cpoly = conv(a, p) + conv(b, q);

​    

​    % 3. 分解为f(s)和h(s)

​    r = roots(cpoly);

​    

​    % 按实部排序（最稳定的根给f(s)）

​    [~, idx] = sort(real(r));

​    r_sorted = r(idx);

​    

​    % f(s)取前n个最稳定的根

​    f_roots = r_sorted(1:n);

​    f = poly(f_roots);

​    

​    % h(s)取剩余的根

​    h_roots = r_sorted(n+1:end);

​    if isempty(h_roots)

​        h = 1;

​    else

​        h = poly(h_roots);

​    end

​    

​    % 4. 构建M, N, X, Y

​    M = tf(a, f);

​    N = tf(b, f);

​    X = tf(p, h);

​    Y = tf(q, h);

​    

​    % 5. 处理b(s)的分解

​    b_roots = roots(b);

​    stable_roots = b_roots(real(b_roots) < 0);

​    unstable_roots = b_roots(real(b_roots) >= 0);

​    

​    b_s = poly(stable_roots);

​    b_u = poly(unstable_roots);

​    

​    if isempty(b_s)

​        b_s = 1;

​    end

​    if isempty(b_u)

​        b_u = 1;

​    end

​    

​    b_u_0 = polyval(b_u, 0);

​    

​    % 6. 计算Q1

​    deg_f = length(f) - 1;

​    deg_b_s = length(b_s) - 1;

​    min_k = deg_f - deg_b_s;

​    

​    current_k = length(alphas);

​    if current_k < min_k

​        warning('k值不足: 当前k=%d, 需要k≥%d。自动调整k值。', current_k, min_k);

​        additional_alphas = 5 * ones(1, min_k - current_k);

​        alphas = [alphas, additional_alphas];

​    end

​    

​    denominator_poly = conv(b_s, [b_u_0]);

​    Q1_base = tf(f, denominator_poly);

​    

​    for a_i = alphas

​        Q1_base = Q1_base * tf(a_i, [1 a_i]);

​    end

​    

​    Q1 = minreal(Q1_base);

​    

​    % 7. 计算C1和C2

​    C1 = minreal(Q1/(X - N*Q2));

​    C2 = minreal((Y + M*Q2)/(X - N*Q2));

end