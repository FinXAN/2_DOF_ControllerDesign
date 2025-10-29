

Test Case 1 

 $$ P(s)=\frac{(s-1)}{(s+2)(s^2 +2s+5)} $$ 

-  n = 2 （分母阶数）deg f(s) = 2 
-  稳定零点阶数 deg bs(s) = 0 
-  k >= 2 
```matlab
syms s

b = [1 -1];
a = [1 4 9 10];
P = tf(b,a);
```

 $$ C_0 \left(s\right)=\frac{q\left(s\right)}{p\left(s\right)} $$ 
```matlab
q = [10 8];
p = [1 0];
C0 = tf(q,p);
```

 $$ c\left(s\right)=a\left(s\right)p\left(s\right)+b\left(s\right)q\left(s\right)=f\left(s\right)h\left(s\right),\deg \left(f\right)=\deg \left(a\right),\deg \left(h\right)=\deg \left(p\right) $$ 
```matlab
c = poly2sym(a,s)*poly2sym(p,s)+poly2sym(b,s)*poly2sym(q,s);
% c = sym2poly(c);
factoered = factor(c);
f = sym2poly(factoered(2));
h = sym2poly(factoered(1));
```

 $$ M\left(s\right)=\frac{a\left(s\right)}{f\left(s\right)},N\left(s\right)=\frac{b\left(s\right)}{f\left(s\right)},X\left(s\right)=\frac{p\left(s\right)}{h\left(s\right)},Y\left(s\right)=\frac{q\left(s\right)}{h\left(s\right)} $$ 
```matlab
M = tf(a,f);
N = tf(b,f);
X = tf(P,h);
Y = tf(q,h);
```

 $$ Q_1 \left(s\right)=\frac{f\left(s\right)}{b_s \left(s\right)b_u \left(0\right)}\prod_{i=1}^k \frac{\alpha_i }{s+\alpha_i } $$ 

 $$ \frac{f\left(s\right)}{b_s \left(s\right)b_u \left(0\right)} $$ 
```matlab
k = 2
bs = 1;
bu = [0 -1];

bs = poly2sym(bs,s);
bu = poly2sym(bu,s);
leftHalf = tf(f,sym2poly(bs*bu));
```

 $$ \sum_{i=1}^k \frac{\alpha_i }{s+\alpha_i \;} $$ 
```matlab
alpha = -1;
num = 1;
denum = 1;
for i = 1:k
    num = num * alpha;
    denum = denum * (s - alpha);
    alpha = alpha-1;
end

% num = sym2poly(num);
denum = sym2poly(denum);
rihgtHalf = tf(num,denum);
```

 $$ Q_1 \left(s\right)=\frac{f\left(s\right)}{b_s \left(s\right)b_u \left(0\right)}\sum_{i=1}^k \frac{\alpha_i }{s+\alpha_i } $$ 
```matlab
Q1 = series(leftHalf,rihgtHalf);
G = series(N,Q1);
step(G);
```
