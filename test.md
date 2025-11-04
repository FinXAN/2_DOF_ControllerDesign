

Test Case

 $$ P(s)=\frac{2s+10}{s^2 (s+1)}=\frac{b(s)}{a(s)} $$ 

-  n = 3 （分母阶数）deg f(s) = 3 
-  稳定零点阶数 $\deg \left(b_s \left(s\right)\right)=1$ 
-  k >= 2 
```matlab
syms s
b = [2 10];
a = [1 1 0 0];
P = tf(b,a);
```

 $$ C_0 \left(s\right)=\frac{q\left(s\right)}{p\left(s\right)} $$ 
```matlab
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
```

 $$ c\left(s\right)=f\left(s\right)h\left(s\right) $$ 
```matlab
n = numel(a) - 1;       % deg a = 3
m = numel(p) - 1;       % deg p = 0
% 选前 n 个根构成 f，其余为 h
f = poly(r(1:n));
h = poly(r(n+1:end));
```

 $$ c\left(s\right)=a\left(s\right)p\left(s\right)+b\left(s\right)q\left(s\right)=f\left(s\right)h\left(s\right),\deg \left(f\right)=\deg \left(a\right),\deg \left(h\right)=\deg \left(p\right) $$ 

 $$ M\left(s\right)=\frac{a\left(s\right)}{f\left(s\right)},N\left(s\right)=\frac{b\left(s\right)}{f\left(s\right)},X\left(s\right)=\frac{p\left(s\right)}{h\left(s\right)},Y\left(s\right)=\frac{q\left(s\right)}{h\left(s\right)} $$ 
```matlab
% 验证：deg f = n, deg h = m
assert(length(f)-1 == n, 'deg f != n');
assert(length(h)-1 == m, 'deg h != m');
M = tf(a,f);
N = tf(b,f);
X = tf(p,h);
Y = tf(q,h);
```

 $$ Q_1 \left(s\right)=\frac{f\left(s\right)}{b_s \left(s\right)b_u \left(0\right)}\prod_{i=1}^k \frac{\alpha_i }{s+\alpha_i } $$ 

 $$ \frac{f\left(s\right)}{b_s \left(s\right)b_u \left(0\right)} $$ 
```matlab
% 根据理论修复Q1计算
alphas = [100 30 10 5];   % k=4 for strictly proper

b_roots = roots(b);
stable_roots = b_roots(real(b_roots) < 0);  
unstable_roots = b_roots(real(b_roots) >= 0); 

% 构建b_s(s)和b_u(s)
b_s = poly(stable_roots);    
b_u = poly(unstable_roots);  
b_u_0 = polyval(b_u, 0);

% Q1 = f(s)/(b_s(s)·b_u(0)) · ∏(α_i/(s+α_i))
% 先计算 f(s)/(b_s(s)·b_u(0))
numerator_part = tf(f, 1);
denominator_part = tf(conv(b_s, [b_u_0]), 1);
Q1_base = numerator_part / denominator_part;

% 然后乘以抵消项
for a_i = alphas
    Q1_base = Q1_base * tf(a_i, [1 a_i]);
end

Q1 = Q1_base;
Q2 = tf(1, [1 10]);

```

 $$ T\left(P\right)=\left\lbrack \frac{Q_1 \left(s\right)}{X\left(s\right)-N\left(s\right)Q_2 \left(s\right)},\frac{Y\left(s\right)+M\left(s\right)Q_2 \left(s\right)}{X\left(s\right)-N\left(s\right)Q_2 \left(s\right)}\right\rbrack $$ 
```matlab
C1 = minreal(Q1/(X-N*Q2));
C2  = minreal((Y+M*Q2)/(X-N*Q2));
```
