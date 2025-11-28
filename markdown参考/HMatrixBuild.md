
```matlab
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

构建H矩阵

```matlab
J = [-1,0;0,1];   %这个是反对角矩阵

%下面俩个来自于 deg = 2, a(s) = s^2 + 0s + 0, f0 = 1, f1 = 0, f2 = 0
as = [1,0,0];
[L_a,U_a] = create_LU_matrices(as,2)
```

```matlabTextOutput
L_a = 2x2
     1     0
     0     1

U_a = 2x2
     0     0
     0     0

```

```matlab
%下面俩个来自于 b(s) = 1
bs = [0,0,1];
[L_b,U_b] = create_LU_matrices(bs,2)
```

```matlabTextOutput
L_b = 2x2
     0     0
     0     0

U_b = 2x2
     1     0
     0     1

```

```matlab

%下面俩个来自于 d(s) = s^2+sqrt(2)s + 1
ds = [1,sqrt(2),1];
[L_d,U_d] = create_LU_matrices(ds,2)
```

```matlabTextOutput
L_d = 2x2
1.0000         0
2. 4142    1.0000

U_d = 2x2
1.0000    1.4142
         0    1.0000

```

 $$ H= $$ $$ J\;L_d^{-1} J\;\left\lbrack L_b J-L_a J\right\rbrack {\left\lbrack \begin{array}{cc} L_a  & L_b \newline U_a  & U_b  \end{array}\right\rbrack }^{-1} \left\lbrack \begin{array}{c} L_d \newline U_d  \end{array}\right\rbrack $$ 

first = $J\;L_d^{-1} J$ 


second = $\left\lbrack L_b J-L_a J\right\rbrack$ 


third = ${\left\lbrack \begin{array}{cc} L_a  & L_b \newline U_a  & U_b  \end{array}\right\rbrack }^{-1}$ 


fourth = $\left\lbrack \begin{array}{c} L_d \newline U_d  \end{array}\right\rbrack$ 

```matlab
J = create_J_matrix(2)
```

```matlabTextOutput
J = 2x2
    -1     0
     0     1

```

```matlab
first = J * inv(L_d) * J
```

```matlabTextOutput
first = 2x2
1.0000         0
2. 4142    1.0000

```

```matlab
second = [L_b *J, - L_a * J]
```

```matlabTextOutput
second = 2x4
     0     0     1     0
     0     0     0    -1

```

```matlab
third = inv([L_a,L_b;U_a,U_b])
```

```matlabTextOutput
third = 4x4
     1     0     0     0
     0     1     0     0
     0     0     1     0
     0     0     0     1

```

```matlab
fourth = [L_d;U_d]
```

```matlabTextOutput
fourth = 4x2
1.0000         0
2. 4142    1.0000
3. 0000    1.4142
         0    1.0000

```

```matlab
H = first * second * third * fourth
```

```matlabTextOutput
H = 2x2
1.0000    1.4142
2. 4142    1.0000

```
