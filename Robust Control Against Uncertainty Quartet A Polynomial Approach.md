## **Algorithm 1** 

**Step 1**: 

(*Spectral factorization*) Find a stable polynomial

$d(s)=d_0s^n+d_1s^{n-1}+\cdots+d_n$

such that

$a(-s)a(s)+b(-s)b(s)=d(-s)d(s).$

**Step 2**: 

(*Matrix construction*) Construct

$\boldsymbol{H}=\boldsymbol{J}\boldsymbol{L}_d^{-1}\boldsymbol{J}\begin{bmatrix}\boldsymbol{L}_b\boldsymbol{J}-\boldsymbol{L}_a\boldsymbol{J}\end{bmatrix}\begin{bmatrix}\boldsymbol{L}_a&\boldsymbol{L}_b\\\boldsymbol{U}_a&\boldsymbol{U}_b\end{bmatrix}^{-1}\begin{bmatrix}\boldsymbol{L}_d\\\boldsymbol{U}_d\end{bmatrix}.$

**Step 3**: 

(*Eigen-computation*) Find the eigenvalue of *H* whose magnitude equals to the spectral radius *ρ*(*H*). Let *e* be an eigenvector corresponding to this eigenvalue.

**Step 4**: (*Pole placement*) Compute

$\begin{bmatrix}\boldsymbol{p}\\\boldsymbol{q}\end{bmatrix}=\begin{bmatrix}\boldsymbol{L}_a&\boldsymbol{L}_b\\\boldsymbol{U}_a&\boldsymbol{U}_b\end{bmatrix}^{-1}\begin{bmatrix}\boldsymbol{L}_d\\\boldsymbol{U}_d\end{bmatrix}\boldsymbol{e}\quad\mathrm{and}\quad\begin{cases}p(s)=\begin{bmatrix}s^{n-1}&s^{n-2}&\cdots&1\end{bmatrix}\boldsymbol{p}\\q(s)=\begin{bmatrix}s^{n-1}&s^{n-2}&\cdots&1\end{bmatrix}\boldsymbol{q}&\end{cases}.$

An optimal controller is given by $C_{\mathrm{opt}}(s)=\frac{q(s)}{p(s)}.$

**Step 5**:  (*Optimal robustness margin computation*)

$\alpha(P(s))=\frac{1}{\sqrt{1+\rho^2(H)}}.$

Notice that only basic matricial and polynomial manipulations, such as spectral factorization, eigenvalue decomposition and matrix inversion are required in the algorithm above. See Section 3.2 for an illustrative example of applying the algorithm. Whereas **Steps 2, 3, and 5** in Algorithm 1 are concerned with the optimal control design, **Steps 1 and 4** are standard and well known, as we elaborate below. Denoted by  the set of all the polynomials with real coefficients and of degree *n*. That is, for *d*(*s*) ∈ $\mathscr{P}_{n}$ it holds $d(s)=d_{0}s^{n}+d_{1}s^{n-1}+\cdots+d_{n}$ with $d_0\neq0$ . This polynomial

*d*(*s*) is said to be stable if all its roots have negative real parts.

Let the plant *P*(*s*) be given as in (5). Observe that the polynomial

$a(-s)a(s)+b(-s)b(s)$                   (6) 

is self-conjugate, i.e., its conjugate coincides with itself. Consequently, if *z* is a root of this polynomial, then so is -$z$. Together with the coprimeness of *a*(*s*) and *b*(*s*), it follows that this polynomial has no roots on the imaginary axis and all its roots are symmetrical about the imaginary axis. **Step 1** in Algorithm 1 can be carried out by first solving for the roots of the polynomial in (6) and then obtaining a stable polynomial ${d(s)}\in\mathscr{P}_n$ such that 

$a(-s)a(s)+b(-s)b(s)=d(-s)d(s)$        (7)



This process is known as the spectral factorization [Kailath, 1980, Section 3.4], [Qiu and Zhou, 2009, Section 8.1].

Given two coprime polynomials with real coefficients *p*(*s*) and *q*(*s*), by defining

$C(s):=\frac{q(s)}{p(s)}$     (8)

we know from the definition of the Gang of Four transfer matrix *P*(*s*)#*C*(*s*) that the closed-loop poles are the roots of the characteristic polynomial

$a(s)p(s)+b(s)q(s)$

One way to obtain *p*(*s*) and *q*(*s*) is via the pole placement method as follows. Let *e*(*s*) ∈ $\mathscr{P}_{n-1}$ be a stable polynomial. By solving the following polynomial Diophantine equation [Kailath, 1980, Section 4.5] [Qiu and Zhou, 2009, Section 3.6]

$a(s)p(s)+b(s)q(s)=d(s)e(s)$      (9)

we obtain *p*(*s*) and *q*(*s*) with $\max\{\deg p(s),\deg q(s)\}\leq n-1$. A controller *C*(*s*) defined as in (8) then places the closed-loop poles at the roots of *d*(*s*)*e*(*s*). Such a process is called the pole placement design, and the resulting *C*(*s*) is called a pole placement controller. In particular, equating the coefficients in (9) yields the following system of linear equations:

$\begin{bmatrix}L_a&L_b\\U_a&U_b\end{bmatrix}\begin{bmatrix}p\\q\end{bmatrix}=\begin{bmatrix}L_d\\U_d\end{bmatrix}\boldsymbol{e}$      (10)

where the elements in *p* and *q* are the unknowns. The matrix

$\begin{bmatrix}\boldsymbol{L}_a&\boldsymbol{L}_b\\\boldsymbol{U}_a&\boldsymbol{U}_b\end{bmatrix}$

is called a Sylvester’s resultant matrix [Qiu and Zhou, 2009, Section 3.6], which is a 2*n*-by-2*n* nonsingular matrix as *a*(*s*) and *b*(*s*) are coprime. By inverting this matrix as in **Step 4** of Algorithm 1, we obtain the solution to equation (9), as well as the pole placement controller.

It is now obvious that **Steps 2 and 3** of Algorithm 1 serve the purpose of computing a partial set of the closed-loop poles, based on which the pole placement controller resulting from **Step 4** gives rise to an optimally robust controller. The proof of this fact is deferred to Section 4.

## **3.2 An Illustrative Example**

Here we revisit the simple example of a double integrator and apply Algorithm 1 to

obtain an optimally robust controller.

*Example 1.* Let

$P(s)=\frac{1}{s^2}$

Objective: find an optimal controller *C*(*s*) such that $\|P(s)\#C(s)\|_\infty$ is minimized with Algorithm 1.

1. (*Spectral factorization*)

$s^4+1=d(-s)d(s).$

This gives $d(s)=s^{2}+\sqrt2s+1$

2. (*Matrix computation*) We can compute that

$\boldsymbol{H}=\begin{bmatrix}1&\sqrt{2}\\\sqrt{2}&1\end{bmatrix}$

3. (*Eigen-computation*) The eigenvalues of *H* are $1\pm\sqrt{2}.$. The eigenvalue with the largest magnitude is $1+\sqrt{2}$ and the corresponding eigenvector satisfies

$\begin{bmatrix}1&\sqrt{2}\\\sqrt{2}&1\end{bmatrix}\boldsymbol{e}=(1+\sqrt{2})\boldsymbol{e}.$

This gives $\mathrm{e}=\begin{bmatrix}1\\1\end{bmatrix}$. Thus,

$e(s)=\begin{bmatrix}s&1\end{bmatrix}e=s+1$

4. (*Pole placement*) We obtain *p*(*s*) = $s+1+\sqrt{2}$ and *q*(*s*) = $(1+\sqrt{2})s+1 $ from

$\begin{bmatrix}\boldsymbol{p}\\\boldsymbol{q}\end{bmatrix}=\begin{bmatrix}\boldsymbol{L}_a&\boldsymbol{L}_b\\\boldsymbol{U}_a&\boldsymbol{U}_b\end{bmatrix}^{-1}\begin{bmatrix}\boldsymbol{L}_d\\\boldsymbol{U}_d\end{bmatrix}\boldsymbol{e}=\begin{bmatrix}1&0&0&0\\0&1&0&0\\0&0&1&0\\0&0&0&1\end{bmatrix}^{-1}\begin{bmatrix}1&0\\\sqrt{2}&1\\1&\sqrt{2}\\0&1\end{bmatrix}\begin{bmatrix}1\\1\end{bmatrix}=\begin{bmatrix}1\\1+\sqrt{2}\\1+\sqrt{2}\\1\end{bmatrix}.$

An optimally robust controller is then given by

$C_{\mathrm{opt}}(s)=\frac{(1+\sqrt{2})s+1}{s+1+\sqrt{2}}$

5. (*Optimal robustness margin computation*)

$\alpha(P(s))=\frac{1}{\sqrt{4+2\sqrt{2}}}$

## *3.3 The Nongeneric Case*

In Step 3 of Algorithm 1, the generic case where H admits a unique eigenvalue of magnitude ρ(H) is dealt with. Here we mention without proof a method to handle the singular case where H has multiple eigenvalues of magnitude ρ(H). This will not be pursued further elsewhere in this chapter. First we introduce some notation. For a square matrix A ∈ $R^{n×n}$ , denote by $λ_{k}(A)$, k = 1,2,...,n its k-th eigenvalue counting multiplicity, ordered according to

$|\lambda_1(\boldsymbol{A})|=\cdots=|\lambda_\nu(\boldsymbol{A})|>|\lambda_{\nu+1}(\boldsymbol{A})|\geq\cdots\geq|\lambda_n(\boldsymbol{A})|.$.

The spectral radius of H, ρ(H), is hence $|λ_1(H)|$. Let the number of the eigenvalues of magnitude ρ(H) be m(H) := v > 1.

It can be shown using the spectral factorization relation in Step 1 of Algorithm 1 that H is diagonalizable and all its eigenvalues are real, hence either or both of ρ(H) and −ρ(H) are eigenvalues of H. If ρ(H) is an eigenvalue, let $\mathcal{E}_1$ be the corresponding eigenspace; otherwise ${\mathcal{E}}_{1}$ = {0}. Similarly, let $\mathcal{E}_2$ be the eigenspace corresponding to −ρ(H). Then Algorithm 1 with Step 3 replaced by Step 3∗ below yields an optimally robust controller whose order is no larger than n−m(H):

Step 3∗ : (*Eigen-computation*) Find $0\neq e\in \mathcal{E}_1\cup \mathcal{E}_2$ such that the degree of e(s) is minimized, where

$$e(s)=\left[s^{n-1}\ s^{n-2}\ \dots\ 1\right]e.$$

The following example of a special all-pass system illustrates how we utilize the algorithm when m(H) > 1.

*Example 2.* Consider the following all-pass plant

$$P(s)=\frac{(s-1)(s-2)(s-3)}{(s+1)(s+2)(s+3)}=\frac{s^{3}-6s^{2}+11s-6}{s^{3}+6s^{2}+11s+6}.$$

Objective: find an optimal controller C(s) such that $\|P(s)\#C(s)\|_{\infty}$ is minimized using Algorithm 1 equipped with Step 3∗ above.

- (*Spectral factorization*)

$d(s)=\sqrt{2}(s+1)(s+2)(s+3)=\sqrt{2}(s^{3}+6s^{2}+11s+6)$.

- (*Matrix computation*) We can compute that

$$\mathbf{H}={\left[\begin{array}{l l l}{-1}&{0}&{0}\\ {0}&{1}&{0}\\ {0}&{0}&{-1}\end{array}\right]}\ .$$

- (*Eigen-computation*) The eigenvalues of H are 1, −1 and −1, all of which have magnitude 1. Hence m(H) = n = 3. The eigenspaces corresponding to eigenvalues 1 and −1 are, respectively,
  Robust Control against Uncertainty Quartet: A Polynomial Approach 15

$${\mathcal{E}}_{1}=\operatorname{span}\begin{bmatrix}0\\ 1\\ 0\end{bmatrix}\quad{\mathrm{and}}\quad{\mathcal{E}}_{2}=\operatorname{span}\begin{bmatrix}1&0\\ 0&0\\ 0&1\end{bmatrix}.$$

The vector $\mathbf{0}\neq\mathbf{e}\in\mathcal{E}_1\cup\mathcal{E}_2$ such that the degree of e(s) is minimized is given by

$\mathbf{e}=\begin{bmatrix}0\\ 0\\ 1\end{bmatrix},\ \ \text{whereby}\ \ e(s)=1,\ \ \text{and}\ \ \deg e(s)=0=n-m(\mathbf{H}).$

4. (*Pole placement*) We obtain p(s) =  $\sqrt{2}$ and q(s) = 0 from

$$\begin{bmatrix}\boldsymbol{p}\\\boldsymbol{q}\end{bmatrix}=\begin{bmatrix}1&0&0&1&0&0\\6&1&0&-6&1&0\\11&6&1&11&-6&1\\6&11&6&-6&11&-6\\0&6&11&0&-6&11\\0&0&6&0&0&-6\end{bmatrix}^{-1}\begin{bmatrix}\sqrt{2}&0&0\\6\sqrt{2}&\sqrt{2}&0\\11\sqrt{2}&6\sqrt{2}&\sqrt{2}\\6\sqrt{2}&11\sqrt{2}&6\sqrt{2}\\0&6\sqrt{2}&11\sqrt{2}\\0&0&6\sqrt{2}\end{bmatrix}\begin{bmatrix}0\\0\\1\end{bmatrix}=\begin{bmatrix}0\\0\\\sqrt{2}\\0\\0\\0\end{bmatrix}$$

Hence, an optimally robust controller is given by

$$C_{\mathrm{opt}}(s)=0.$$

- (*Optimal robustness margin computation*)

$$\alpha(P(s))={\frac{1}{\sqrt{2}}}.$$

## 4 Proof of Optimality

The purpose of this section is to prove that the controller Copt(s) obtained from Algorithm 1 is optimal, in the sense that it satisfies

$$C_{\mathrm{opt}}(s)={\underset{C(s)}{\operatorname{arg\,min}}}\,\|P(s)\#C(s)\|_{\infty}.$$