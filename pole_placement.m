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
