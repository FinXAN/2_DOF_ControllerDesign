function [rho_H, dominant_eigenvalue, e, eigenvalues] = compute_eigen_properties(H, normalization_type)
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
