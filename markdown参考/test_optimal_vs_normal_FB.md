

最大裕度的C2设计

 $$ P=\frac{s^4 +3s^3 +3s^2 +5s+4}{{2s}^3 +2s^2 +3s+1} $$ 
```matlab
clear
denum = [1 3 3 5 4]; 
num = [2 2  3 1];    
Plant = tf(num,denum);
Q2=TwoDOFFeedbackTuningFunction(num,denum)
[Q1,C1,C2] = TwoDOFtuningFunction(num,denum,Q2);
T = (Plant*C1)/(1+Plant*C2);
step(T)
title("Mine");
```

老师C2设计

```matlab
clear
denum = [1 3 3 5 4]; 
num = [2 2 3 1];  
Plant = tf(num,denum);
Q2 = opt_poly(Plant)
[Q1,C1,C2] = TwoDOFtuningFunction(num,denum,Q2);
T = (Plant*C1)/(1+Plant*C2);
step(T)
title("reference");
```

Mine

```matlab
clear
denum = [1 2]; 
num = [2 4 3 1];    
Plant = tf(num,denum);
Q2=TwoDOFFeedbackTuningFunction(num,denum);
[Q1,C1,C2] = TwoDOFtuningFunction(num,denum,Q2);
T = (Plant*C1)/(1+Plant*C2);
step(T)
title("Mine");
```

老师C2设计

```matlab
clear
denum = [1 2]; 
num = [2 4 3 1];    
Plant = tf(num,denum);
Q2 = opt_poly(Plant)
[Q1,C1,C2] = TwoDOFtuningFunction(num,denum,Q2);
T = (Plant*C1)/(1+Plant*C2);
step(T)
title("reference");
```

测试

```matlab
% 测试一阶系统
num = [1];
den = [1, 2];
P = tf(num, den);
% 使用你的TwoDOFFeedbackTuningFunction设计Q2
Q2=TwoDOFFeedbackTuningFunction(num,denum);
```

```matlabTextOutput
谱分解无明显误差
```

```matlab
% 测试修正后的函数
[C1, C2] = TwoDOFtuningFunction(num, den, Q2);
```

```matlabTextOutput
Not enough input arguments.

Error in place (line 35)
P = P(:);

Error in TwoDOFtuningFunction (line 21)
    [K, ~, ~] = place(ss(P), desired_poles);
```

```matlab
% 检查闭环稳定性
T = (P*C1)/(1 + P*C2);
step(T);
```
