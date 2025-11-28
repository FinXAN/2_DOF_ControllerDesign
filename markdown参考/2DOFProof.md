The rest of this section is dedicated to the proof of Theorem 4.22. The proof  s constructive, so it also tells how such a 2DOF controller can be designed.

Here we use the parametrization of all stabilizing 2DOF controllers given in Section 3.8.Let the plant

$P(s)=\frac{b(s)}{a(s)}$

be proper with deg a(s)=n and

$C_0(s)=\frac{q(s)}{p(s)}$

where deg p(s)= m be an initial stabilizing feedback controller. The assumption that P(0)≠0 implies that b(0)≠0. Factorize c(s)=a(s)p(s)+b(s)q(s) as c(s)=f(s)h(s) such that deg f(s)=n and deg h(s)=m. Let

$M(s)=\frac{a(s)}{f(s)},\quad N(s)=\frac{b(s)}{f(s)},\quad X(s)=\frac{p(s)}{h(s)},\quad Y(s)=\frac{q(s)}{h(s)}.$

It follows from the theory in Section 3.8 that the set of all stabilizing 2DOF controllers is given by

$\mathcal{T}(P)=\left\{\boldsymbol{C}(s)=\begin{bmatrix}\frac{Q_1(s)}{X(s)-N(s)Q_2(s)}&\frac{Y(s)+M(s)Q_2(s)}{X(s)-N(s)Q_2(s)}\end{bmatrix}\right.$ , Q₁(s) and Q₂(s) are two arbitrary stable systems

and the closed-loop transfer function from r(t) to z(t) is

$G(s)=N(s)Q_1(s)=\frac{b(s)}{f(s)}Q_1(s)$

which is independent of Q2(s). This shows that Q2(s) has nothing to do with the step response from r(t)to z(t) and hence can be designed independently. Factorize b(s) as b(s)=b₈(s)b₂(s) where bs(s) is astable polynomial and bz(s) is a polynomial whose roots are the unstable roots of b(s). Construct

$Q_1(s)=\frac{f(s)}{b_s(s)b_u(0)}\prod_{i=1}^k\frac{\alpha_i}{s+\alpha_i}$

We have accomplished two objectives in designing this Q1(s). The first is to cancel the stable zeros of N(s) using the stable poles of Q1(s). The second is to assign all poles of G(s) to negative real numbers. Let us first consider a number of special cases, which cover the majority of the practical cases.

Case 1: $b_u(s)$ is a constant, i.e., the plant is minimum phase.

In this case,

$G(s)=\prod_{i=1}^k\frac{\alpha_i}{(s+\alpha_i)}.$

The statement in Problem 4.18 implies that the step response of G(s) is monotonic and hence is free of overshoot.

Case 2: $b_u(s)$= s \-z with z> 0, i.e., the plant only has one unstable zero z that is real and positive.

$G(s)=\frac{\alpha_1(s-z)}{-z(s+\alpha_1)}\prod_{i=2}^k\frac{\alpha_i}{s+\alpha_i}.$

Case 3: $b_u(s)=s^2-2\zeta\omega_ns+\omega_n^2$ with ζ≥0, wn>0, i.e, the plant only has two complex conjugate or real unstable zeros. In this case, set

$\alpha_1=\alpha_2=\omega_n$

Then

$G(s)=\frac{s^2-2\zeta\omega_ns+\omega_n^2}{(s+\omega_n)^2}\prod_{i=3}^k\frac{\alpha_i}{s+\alpha_i}.$

The conclusion in Problem 4.20.3 implies that z(t) is free of overshoot.

In the general case, we cannot simply extend the design ideas above. We need to follow a different idea. 

Let us set $\alpha_i=\alpha$ for all $i$. Then the tracking error for a unit step reference is:

$E(s)=[1-G(s)]\frac{1}{s}=\left[1-\frac{b_u(s)\alpha^k}{b_u(0)(s+\alpha)^k}\right]\frac{1}{s}=\frac{b_u(0)(s+\alpha)^k-b_u(s)\alpha^k}{b_u(0)(s+\alpha)^ks}$

Notice The numerator is

$b_u(0)[(s+\alpha)^k-\alpha^{k}]-[b_u(s)-b_u({0})]\alpha^{k}\\=b_u(0)s[(s+\alpha)^{k-1}+(s+\alpha)^{k-2}\alpha+\cdots+(s+\alpha)\alpha^{k-2}+\alpha^{k-1}]-[b_u(s)-b_u(0)]\alpha^k$

Also notice that the second term above can be divided by s and hence, ac

cording to the partial fractional expansion formulas in Section A.3,

$\frac{b_u(s)-b_u(0)}{b_u(0)(s+\alpha)^ks}=\frac{\beta_1(\alpha)}{s+\alpha}+\frac{\beta_2(\alpha)}{(s+\alpha)^2}+\cdots+\frac{\beta_k(\alpha)}{(s+\alpha)^k}$

where $\beta(\alpha)$, $i$=1,2,...,k, are polynomial functions of $\alpha$. This shows that 

$E(s)=\frac{\gamma_1(\alpha)}{s+\alpha}+\frac{\gamma_2(\alpha)}{(s+\alpha)^2}+\cdots+\frac{\gamma_k(\alpha)}{(s+\alpha)^k}$

where for $i$ = 1,2,...,k

${\gamma}_{i}(\alpha)=\alpha^{i-1}-\beta_{i}(\alpha)\alpha^{k}=\alpha^{i-1}[1-\beta_{i}(\alpha)\alpha^{k-i+1}].$

As long as $\alpha$ is sufficiently small. $\gamma_i(\alpha)$, $i$=1,2,...,k, are nonnegative, Consequently,

$e(t)=\mathcal{L}^{-1}[E(s)]=\left(\gamma_1(\alpha)+\gamma_2(\alpha)t+\cdots+\frac{\gamma_k(\alpha)}{(k-1)!}t^{k-1}\right)e^{-\alpha t}\sigma(t)$

is always nonnegative, which means that z( t) has no overshoot .

