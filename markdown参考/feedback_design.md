构建H矩阵

```matlab
J = [-1,0;0,1];   %这个是反对角矩阵

%下面俩个来自于 deg = 2, a(s) = s^2 + 0s + 0, f0 = 1, f1 = 0, f2 = 0
L_a = [1,0;0,1]
U_a = [0,0;0,0];

%下面俩个来自于 b(s) = 1
L_b = [0,0;0,0];
U_b = [1,0;0,1];

%下面俩个来自于 d(s) = s^2+sqrt(2)s + 1
L_d = [1,0;sqrt(2),1];
U_d = [1,sqrt(2);0,1];

```

 $$ H= $$ $$ J\;L_d^{-1} J\;\left\lbrack L_b J-L_a J\right\rbrack {\left\lbrack \begin{array}{cc} L_a  & L_b \newline U_a  & U_b  \end{array}\right\rbrack }^{-1} \left\lbrack \begin{array}{c} L_d \newline U_d  \end{array}\right\rbrack $$ 

first = $J\;L_d^{-1} J$ 


second = $\left\lbrack L_b J-L_a J\right\rbrack$ 


third = ${\left\lbrack \begin{array}{cc} L_a  & L_b \newline U_a  & U_b  \end{array}\right\rbrack }^{-1}$ 


fourth = $\left\lbrack \begin{array}{c} L_d \newline U_d  \end{array}\right\rbrack$ 

```matlab
% 正确计算步骤：
step1 = J * inv(L_d) * J;                    % 2×2
step2 = (L_b * J) - (L_a * J);               % 2×2
step3 = inv([L_a,L_b;U_a,U_b]);              % 4×4  
step4 = [L_d;U_d];                           % 4×2
% 现在计算：
temp = step1 * step2;                        % 2×2 × 2×2 = 2×2
temp2 = temp * step3;                        % 2×2 × 4×4 = 2×4  ← 这里维度就不对了！
H = temp2 * step4;                           % 2×4 × 4×2 = 2×2

```
