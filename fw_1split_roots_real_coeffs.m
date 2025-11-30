function [f, h] = split_roots_real_coeffs(roots, n)
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
