

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
```

```matlabTextOutput
谱分解无明显误差
```

```matlab
[Q1,C1,C2] = TwoDOFtuningFunction(num,denum,Q2);
```

```matlabTextOutput
Warning: Cannot keep loop gain above 1 at low frequency and below 1 at high frequency. Try adding integrators to the loop.
Error using assert
deg h != m

Error in TwoDOFtuningFunction (line 35)
assert(length(h)-1 == m, 'deg h != m');
```

```matlab
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
