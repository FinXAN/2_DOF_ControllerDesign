# Robust Control against Uncertainty Quartet: A Polynomial Approach 

Di Zhao, Chao Chen, Sei Zhen Khong and Li Qiu

In memory of Robert Tempo.


#### Abstract

One of the main components of a robust control theory is a quantifiable description of system uncertainty. A good uncertainty description should have three desirable properties. First, it is required to capture important unmodeled dynamics and perturbations. Second, it needs to be mathematically tractable, preferably by using elementary tools. Third, it should lead to a self-contained robust control theory, encompassing analysis and synthesis techniques that are accessible to both researchers and practitioners. While the additive uncertainty and multiplicative uncertainty are two of the most commonly employed uncertainty descriptions in systems modelling and control, they come up short of fulfilling the requirements above. In this chapter, we introduce the uncertainty quartet, a.k.a. the $+-\times \div$ uncertainty (as is simpler to pronounce in oriental languages), which combines in a unifying framework the additive, multiplicative, subtractive and divisive uncertainties. An elementary robust control theory, involving mostly polynomial manipulations, is developed based on the uncertainty quartet. The proposed theory is demonstrated in a case study on controlling an under-sensed and under-actuated linear (USUAL) inverted pendulum system.


## 1 Uncertainty in Dynamical Systems

Model-based control synthesis is ubiquitous in engineering. It involves designing a controller based on a mathematical model of the system to be controlled (a.k.a. the

[^0]plant). Every model, irrespective of its complexity, can at best approximate the dynamics of a real system. In other words, uncertainty is inherent to any mathematical model of a system. An uncertainty description or model provides a useful means to characterizing certain unmodeled dynamics of and unmeasured perturbations on a system. In what follows, we review several uncertainty models that have been widely adopted in practice, and discuss their advantages and shortcomings with the aid of an illustrative example involving a double integrator. Moreover, we introduce a powerful uncertainty description, known as the uncertainty quartet, that can be used to model a large class of uncertainties. A robust control theory based on the uncertainty quartet is subsequently developed in the succeeding sections.

### 1.1 Common Uncertainty Descriptions

The additive and multiplicative uncertainty descriptions constitute two of the most well-studied models in robust control. As an illustration, consider a nominal double integrator system

$$
P(s)=\frac{1}{s^{2}}
$$

which may model an ideal rigid body undergoing a forced linear motion. The real system, however, would have an elastic body. The dynamics arising from the presence of elasticity are not captured in this model and may correspond to an additive damped oscillatory term:

$$
\tilde{P}(s)=P(s)+\frac{\delta \omega_{n}^{2}}{s^{2}+2 \zeta \omega_{n} s+\omega_{n}^{2}},
$$

where $\delta$ denotes a small gain, $\zeta$ the damping ratio and $\omega_{n}$ the natural frequency, which can all be uncertain. The value of $\delta$ provides a quantification of the difference between the nominal system $P(s)$ and its perturbed model $\tilde{P}(s)$. The additive uncertainty description of the form

$$
\tilde{P}(s)=P(s)+\Delta_{+}(s)
$$

can be used to model the aforementioned uncertainty satisfactorily. Observe that in this example, $\Delta_{+}(s)$ is stable and has a small magnitude response as determined by the small parameter $\delta$.

The use of the additive uncertainty model alone can be restrictive, as we explain below. Suppose that the double integrator is subject to an uncertain gain instead and the real system takes the form

$$
\tilde{P}_{1}(s)=\frac{1+\delta}{s^{2}} \quad \text { or } \quad \tilde{P}_{2}(s)=\frac{1}{(1+\delta) s^{2}}
$$

where $\delta$ denotes a small parameter. To model $\tilde{P}_{i}(s)$ using the additive uncertainty model, one would have to let

$$
\Delta_{+}(s)=\frac{\delta}{s^{2}} \quad \text { or } \quad \Delta_{+}(s)=\frac{-\delta}{(1+\delta) s^{2}}
$$

In this case, both the uncertainty terms above are unstable, which are in no sense small since they both have infinite induced gains. In order to model the aforementioned perturbations with reasonably small uncertainties, we appeal to alternative uncertainty models having the multiplicative form

$$
\tilde{P}_{1}(s)=\left(1+\Delta_{\times}(s)\right) P(s)
$$

or the divisive form (a.k.a. the relative form)

$$
\tilde{P}_{2}(s)=\frac{P(s)}{1+\Delta \div(s)}
$$

With respect to these models, we have $\Delta_{\times}(s)=\delta$ and $\Delta_{\div}(s)=\delta$, both of which are stable and small in magnitudes, as desired.

The following further demonstrates that the uncertainty descriptions covered above are still inadequate from a practical point of view. Due to the existence of a small stiffness in the rigid body motion, suppose the real system takes the form

$$
\tilde{P}(s)=\frac{1}{s^{2}+\varepsilon^{2}}
$$

where $\varepsilon$ is a small parameter. In this case, it can be verified that applying the additive, multiplicative and divisive models would result in an uncertainty term that is unstable. On the contrary, the subtractive form (a.k.a. the feedback form)

$$
\tilde{P}(s)=\frac{P(s)}{1+\Delta_{-}(s) P(s)}
$$

gives $\Delta_{-}(s)=\varepsilon^{2}$, which is stable and small in magnitude.

### 1.2 The Uncertainty Quartet

By integrating the additive, subtractive, multiplicative, and divisive uncertainty models within a unifying framework, we arrive at the following uncertainty description

$$
\begin{equation*}
\tilde{P}(s)=\frac{\left(1+\Delta_{\times}(s)\right) P(s)+\Delta_{+}(s)}{1+\Delta_{\div}(s)+\Delta_{-}(s) P(s)} \tag{1}
\end{equation*}
$$

see the block diagram in Fig. 1 for a depiction of $\tilde{P}(s)$ as a mapping from $\tilde{u}$ to $\tilde{y}$. We call the band of the four uncertainties the uncertainty quartet (or the $+-\times \div$ uncertainty), and the 2-by-2 transfer matrix

$$
\Delta(s):=\left[\begin{array}{cc}
\Delta_{\div}(s) & \Delta_{-}(s) \\
\Delta_{+}(s) & \Delta_{\times}(s)
\end{array}\right]
$$

the uncertainty quartet matrix. It is straightforward to see that (1) gives rise to a versatile form that can be used to model a wide class of uncertainties.

To motivate the utility of the uncertainty quartet, let us revisit the example of the double integrator. Suppose the real system has dynamics of the form

$$
\tilde{P}(s)=\frac{1+\delta_{2}}{s^{2}+\delta_{1} s+\varepsilon^{2}},
$$

where $\varepsilon^{2}$ is a small stiffness term, $\delta_{1}$ a small damping coefficient and $\delta_{2}$ a small uncertain gain. It can be verified that using only the additive and multiplicative forms of uncertainty would result in unstable $\Delta_{+}(s)$ and $\Delta_{\times}(s)$ regardless of the values of $\varepsilon^{2}, \delta_{1}$ and $\delta_{2}$. Likewise, adopting only the relative and the feedback form of the uncertainty would give rise to unstable $\Delta_{\div}(s)$ and $\Delta_{-}(s)$. On the other hand, if we characterize $\tilde{P}(s)$ with the uncertainty quartet by applying equation (1), we obtain

$$
\Delta(s)=\left[\begin{array}{cc}
\Delta_{\div}(s) & \Delta_{-}(s) \\
\Delta_{+}(s) & \Delta_{\times}(s)
\end{array}\right]=\frac{1}{s+1}\left[\begin{array}{cc}
\delta_{1} & \left(\delta_{1}+\varepsilon^{2}\right) s+\varepsilon^{2} \\
0 & \delta_{2}
\end{array}\right] .
$$

Each member in this uncertainty quartet is stable and small in magnitude. This example demonstrates the fact that while each of the individual uncertainty models falls short of providing a satisfactory characterization of the uncertainty, their combination (1) introduces a powerful framework in which we can model various types of perturbations.

![](https://cdn.mathpix.com/cropped/e749505a-107f-4ec0-b869-e0b5d9c5f433-04.jpg?height=491&width=800&top_left_y=1600&top_left_x=640)
Fig. 1 An uncertain system with $+-\times \div$ uncertainty.

Mathematically, the map from $P(s)$ to $\tilde{P}(s)$ is a linear fractional transformation (LFT). In particular, let

$$
\operatorname{LFT}\left(\left[\begin{array}{cc}
T_{11}(s) & T_{12}(s) \\
T_{21}(s) & T_{22}(s)
\end{array}\right], P(s)\right)=\frac{T_{22}(s) P(s)+T_{12}(s)}{T_{11}(s)+T_{21}(s) P(s)},
$$

then

$$
\tilde{P}(s)=\operatorname{LFT}\left(\left[\begin{array}{cc}
1+\Delta_{\div}(s) & \Delta_{-}(s) \\
\Delta_{+}(s) & 1+\Delta_{\times}(s)
\end{array}\right], P(s)\right) .
$$

The study of various uncertainty models has a long history in the field of robust control. The additive, subtractive, multiplicative, and divisive models were covered in such classic books as [Doyle et al., 1990], [Zhou and Doyle, 1998] and revisited more recently in, for example, [Liu and Yao, 2016]; see also the survey paper [Petersen and Tempo, 2014]. The uncertainty quartet unifies all 4 of the aforementioned uncertainties within one powerful framework for robustness analysis and control synthesis. It is worth noting that the notation of $+-\times \div$ uncertainty was first used in [Halsey and Glover, 2005]. It is shown in [Gu and Qiu, 1998] that the uncertainty quartet is closely related to the gap metric and its variations [Zames and El-sakkary, 1980], [Georgiou and Smith, 1990], [Qiu and Davison, 1992a], [Qiu and Davison, 1992b], [Vinnicombe, 1993], [Georgiou and Smith, 1997]. The uncertainty quartet has also been used to describe the interferences and distortions within a communication channel modeled by a twoport network [Gu and Qiu, 2011], [Zhao and Qiu, 2016]. Moreover, one may relate the uncertainty quartet to the coprime-factor uncertainty [Vidyasagar, 1985], [McFarlane and Glover, 1990], [Georgiou and Smith, 1990], namely, a pair of dynamic uncertainties additive to the coprime factors of a nominal system. It is noteworthy that in the uncertainty quartet, each member acts directly on the input and output of the nominal system, whereas the coprime-factor uncertainty depends on a particular coprime factorization of the nominal system. In addition to the uncertainty quartet, many other types of dynamic uncertainties have been studied over the past decades; see, for instance, [Zhou and Doyle, 1998], [Petersen and Tempo, 2014], [Liu and Yao, 2016], [Lanzon and Papageorgiou, 2009].

### 1.3 Notation

We formalize the notation in this chapter. Let $\boldsymbol{I}$ denote the identity matrix of a proper dimension. Let $\mathscr{R}^{p \times m}$ denote the set of all $p \times m$ proper real-rational transfer function matrices. The set of elements in $\mathscr{R}^{p \times m}$ containing bounded singular values on the imaginary axis is denoted by $\mathscr{R} \mathscr{L}_{\infty}^{p \times m}$ and the set of elements in $\mathscr{R} \mathscr{L}_{\infty}^{p \times m}$ with bounded singular values on the right complex plane $\operatorname{Re} s>0$ is denoted by $\mathscr{R} \mathscr{H}_{\infty}^{p \times m}$. A transfer function $P(s) \in \mathscr{R}^{p \times m}$ is said to be stable if $P(s) \in \mathscr{R} \mathscr{H}_{\infty}^{p \times m}$. Define the set of uncertain systems of size $r \in[0,1)$ centered at $P(s)$ as

$$
\begin{equation*}
\mathscr{B}(P(s), r)=\left\{\operatorname{LFT}(\boldsymbol{I}+\boldsymbol{\Delta}(s), P(s)): \boldsymbol{\Delta}(s) \in \mathscr{R} \mathscr{H}_{\infty}^{2 \times 2},\|\boldsymbol{\Delta}(s)\|_{\infty} \leq r\right\} . \tag{2}
\end{equation*}
$$

Throughout, the superscripts corresponding to the dimensions may be omitted for notational simplicity.

Recall the standard Lebesgue space $\mathscr{L}_{2}$ endowed with the norm $\|\cdot\|_{2}$ and Hardy space $\mathscr{H}_{2} \subset \mathscr{L}_{2}$. The orthogonal complement of $\mathscr{H}_{2}$ in $\mathscr{L}_{2}$ is denoted by $\mathscr{H}_{2}{ }^{\perp}$. In other words, $\mathscr{L}_{2}=\mathscr{H}_{2} \oplus \mathscr{H}_{2}{ }^{\perp}$, where ⊕ denotes the orthogonal sum. For a $G(s) \in \mathscr{R} \mathscr{L}_{\infty}$, we have

$$
\|G(s)\|_{\infty}=\sup _{U(s) \in \mathscr{L}_{2}} \frac{\|G(s) U(s)\|_{2}}{\|U(s)\|_{2}} .
$$

Moreover, if $U_{1}(s) \in \mathscr{H}_{2}$ and $U_{2}(s) \in \mathscr{H}_{2}^{\perp}$, then

$$
\left\|U_{1}(s)+U_{2}(s)\right\|_{2}^{2}=\left\|U_{1}(s)\right\|_{2}^{2}+\left\|U_{2}(s)\right\|_{2}^{2} .
$$

## 2 Robust Closed-Loop Stability

As explained in the last section, uncertainty is intrinsic to every mathematical model of a system. This fact is particularly problematic to model-based control - if a model does not accurately describe the behavior of a system, how can we be certain that a controller designed based on the model will perform well when it is implemented on the system? Feedback, which underlies the field of systems and control, is most commonly adopted to resolve this issue. It is a powerful tool with which we desensitize a dynamical system to the effect of uncertainty. The theory of feedback control, which will be briefly reviewed in this section, has been well studied over recent decades and demonstrated to be effective in many application scenarios [Zhou and Doyle, 1998], [Qiu and Zhou, 2009], [Vinnicombe, 2000]. We begin with the notion of a standard feedback (or closed-loop) system, and define its closed-loop stability. Then we analyze robust closed-loop stability when the plant is subject to uncertainty quartet, based on which we derive a robust stability condition.

![](https://cdn.mathpix.com/cropped/e749505a-107f-4ec0-b869-e0b5d9c5f433-06.jpg?height=244&width=502&top_left_y=1712&top_left_x=794)
Fig. 2 A standard feedback system.

A closed-loop system composed of a plant $P(s) \in \mathscr{R}$ and a feedback controller $C(s) \in \mathscr{R}$ is illustrated in Fig. 2. We denote it by $P(s) \# C(s)$. We say that $P(s) \# C(s)$ is stable if for all exogenous signals $w_{1}, w_{2} \in \mathscr{H}_{2}$, the endogenous signals
$u_{1}, u_{2}, y_{1}, y_{2}$ exist and belong to $\mathscr{H}_{2}$. Intuitively, stability means that the energy within the feedback system stays bounded when it is injected with bounded-energy exogenous signals. It is known that $P(s) \# C(s)$ is stable if and only if the associated Gang of Four transfer matrix

$$
P(s) \# C(s):=\left[\begin{array}{c}
1 \\
P(s)
\end{array}\right](1+P(s) C(s))^{-1}[1 C(s)]=\left[\begin{array}{cc}
\frac{1}{1+P(s) C(s)} & \frac{C(s)}{1+P(s) C(s)} \\
\frac{P(s)}{1+P(s) C(s)} & \frac{P(s) C(s)}{1+P(s) C(s)}
\end{array}\right]
$$

is stable [Åström and Murray, 2008]. Here, both the closed-loop system and its associated Gang of Four transfer matrix are denoted by $P(s) \# C(s)$ for notational simplicity.

![](https://cdn.mathpix.com/cropped/e749505a-107f-4ec0-b869-e0b5d9c5f433-07.jpg?height=532&width=803&top_left_y=921&top_left_x=603)
Fig. 3 A closed-loop system with $+-\times \div$ uncertainty quartet at plant side.

Recall from the preceding section that an uncertainty quartet is useful for modeling a rich class of uncertainties. When model-based feedback control design is performed based on a mathematical model of a plant, it gives rise to a stable nominal closed-loop system. In the following, we analyze the closed-loop stability when the plant is subject to uncertainty quartet and provide a quantification of how much uncertainty is tolerable while the feedback system remains stable. Mathematically, let $P(s) \# C(s)$ be a nominal closed-loop system. We derive an upper bound on $r>0$ such that $\tilde{P}(s) \# C(s)$ is stable for all $\tilde{P}(s) \in \mathscr{B}(P(s), r)$. First recall from (2) that every $\tilde{P}(s) \in \mathscr{B}(P(s), r)$ can be expressed as

$$
\tilde{P}(s)=\operatorname{LFT}\left(\left[\begin{array}{cc}
1+\Delta_{\div}(s) & \Delta_{-}(s) \\
\Delta_{+}(s) & 1+\Delta_{\times}(s)
\end{array}\right], P(s)\right)=\frac{\left(1+\Delta_{\times}(s)\right) P(s)+\Delta_{+}(s)}{1+\Delta_{\div}(s)+\Delta_{-}(s) P(s)},
$$

where

$$
\|\boldsymbol{\Delta}(s)\|_{\infty}=\left\|\left[\begin{array}{cc}
\Delta_{\div}(s) & \Delta_{-}(s) \\
\Delta_{+}(s) & \Delta_{\times}(s)
\end{array}\right]\right\|_{\infty} \leq r .
$$

See Fig. 3 for a depiction of the perturbed closed-loop system $\tilde{P}(s) \# C(s)$. It is shown in [Gu and Qiu, 2011] that the signals within the perturbed closed-loop system satisfy the following relations

$$
\left[\begin{array}{l}
\hat{u}_{1} \\
\hat{y}_{2}
\end{array}\right]=\boldsymbol{\Delta}(s)\left[\begin{array}{l}
u_{1} \\
y_{2}
\end{array}\right] \quad \text { and } \quad\left[\begin{array}{l}
u_{1} \\
y_{2}
\end{array}\right]=P(s) \# C(s)\left[\begin{array}{l}
\hat{u}_{1} \\
\hat{y}_{2}
\end{array}\right] .
$$

As a result, we can equivalently transform the perturbed closed-loop system in Fig. 3 into a standard feedback interconnection of an uncertainty quartet $\boldsymbol{\Delta}(s)$ and the Gang of Four transfer matrix $P(s) \# C(s)$ as shown in Fig. 4. Furthermore, it can be verified that the stability of $\tilde{P}(s) \# C(s)$ is equivalent to that of the closed-loop system in Fig. 4.

![](https://cdn.mathpix.com/cropped/e749505a-107f-4ec0-b869-e0b5d9c5f433-08.jpg?height=299&width=558&top_left_y=898&top_left_x=734)
Fig. 4 An equivalent closed-loop system composed of an uncertainty quartet and the Gang of Four transfer matrix.

Since both open-loop systems $\boldsymbol{\Delta}(s)$ and $P(s) \# C(s)$ are stable, robust stability of the closed-loop system in Fig. 4 can be analyzed by means of the well-known small-gain theorem [Zhou and Doyle, 1998, Theorem 8.1]. In particular, the closedloop system $\boldsymbol{\Delta}(s) \#[P(s) \# C(s)]$ is stable for all $\|\boldsymbol{\Delta}(s)\| \leq r$ if, and only if, $r< \|P(s) \# C(s)\|_{\infty}^{-1}$. Consequently, we have the following robust stability condition.

Theorem 1. Let $r \in[0,1)$. The perturbed closed-loop system $\tilde{P}(s) \# C(s)$ in Fig. 3 is stable for all $\tilde{P}(s) \in \mathscr{B}(P(s), r)$ if and only if

$$
r<\|P(s) \# C(s)\|_{\infty}^{-1} .
$$

By virtue of Theorem 1, it is natural to define $\|P(s) \# C(s)\|_{\infty}^{-1}$ as the robust stability margin of the nominal closed-loop system $P(s) \# C(s)$. The larger the margin is, the more robust the closed-loop system will be against model uncertainties characterized in the form of an uncertainty quartet. An optimal control problem naturally arises from this context. It involves designing a feedback controller $C(s)$ for a nominal plant $P(s)$ such that the stability margin $\|P(s) \# C(s)\|_{\infty}^{-1}$ is maximized, or equivalently, solving the following $\mathscr{H}_{\infty}$ control problem:

$$
\begin{equation*}
\min _{C(s)}\|P(s) \# C(s)\|_{\infty} . \tag{3}
\end{equation*}
$$

The optimally robust stability margin is thus given by

$$
\begin{equation*}
\alpha(P(s)):=\left(\min _{C(s)}\|P(s) \# C(s)\|_{\infty}\right)^{-1} . \tag{4}
\end{equation*}
$$

## 3 Optimally Robust Controller Design

In this section, our aim is to derive an optimally robust controller $C(s)$ that minimizes $\|P(s) \# C(s)\|_{\infty}$. This is an $\mathscr{H}_{\infty}$ optimal control problem. It has favourable properties and can be solved efficiently using state-space methods [Doyle et al., 1989], [McFarlane and Glover, 1990] based on algebraic Riccati equations. Given the simplicity of our setup, formulated in terms of scalar transfer functions, we provide below a more straightforward and efficient alternative to solving the optimization problem via a polynomial approach.

It is worth noting that a certain polynomial method was proposed in [Liang and Qiu, 2009], [Qiu and Zhou, 2009, Chapter 9], where the $\mathscr{H}_{\infty}$ optimal control problem is solved by calculating a Hankel matrix based on special bases. Another similar polynomial method was proposed in [Kanno, 2003] based on solving polynomial equations. By contrast, in this chapter, we propose an even simpler alternative polynomial method, involving only elementary matricial and polynomial manipulations.

### 3.1 Main Algorithm

First we introduce some notation. Consider an arbitrary polynomial with real coefficients

$$
f(s)=f_{0} s^{n}+f_{1} s^{n-1}+\cdots+f_{n}
$$

whose degree, denoted by $\operatorname{deg} f(s)$, is no larger than $n$. Correspondingly to the polynomial $f(s)$, define

$$
\boldsymbol{f}:=\left[\begin{array}{c}
f_{0} \\
\vdots \\
f_{n}
\end{array}\right], \quad \boldsymbol{L}_{f}:=\left[\begin{array}{cccc}
f_{0} & 0 & \cdots & 0 \\
f_{1} & f_{0} & \ddots & \vdots \\
\vdots & \ddots & \ddots & 0 \\
f_{n-1} & \cdots & f_{1} & f_{0}
\end{array}\right] \quad \text { and } \quad \boldsymbol{U}_{f}:=\left[\begin{array}{cccc}
f_{n} & f_{n-1} & \cdots & f_{1} \\
0 & f_{n} & \ddots & \vdots \\
\vdots & \ddots & \ddots & f_{n-1} \\
0 & \cdots & 0 & f_{n}
\end{array}\right] .
$$

Let $\boldsymbol{J}$ be a sign matrix, defined as

$$
\boldsymbol{J}:=\left[\begin{array}{ccccc}
(-1)^{n-1} & & & & \\
& \ddots & & \\
& & & -1 & \\
& & & & 1
\end{array}\right]
$$

The matrices $\boldsymbol{L}_{f}, \boldsymbol{U}_{f}$ and $\boldsymbol{J}$ are helpful in transforming a polynomial equation into a system of linear equations. As we shall see, such a transformation plays an important role in the proposed polynomial approach.

Suppose we are given an $n$th order plant

$$
\begin{equation*}
P(s)=\frac{b(s)}{a(s)}=\frac{b_{0} s^{n}+b_{1} s^{n-1}+\cdots+b_{n}}{a_{0} s^{n}+a_{1} s^{n-1}+\cdots+a_{n}}, \tag{5}
\end{equation*}
$$

where $a_{0} \neq 0$, and $a(s)$ and $b(s)$ are coprime. The following algorithm computes an optimally robust controller

$$
C_{\mathrm{opt}}(s)=\underset{C(s)}{\arg \min }\|P(s) \# C(s)\|_{\infty}
$$

Algorithm 1 Optimally Robust Controller Design
Step 1: (Spectral factorization) Find a stable polynomial

$$
d(s)=d_{0} s^{n}+d_{1} s^{n-1}+\cdots+d_{n}
$$

such that

$$
a(-s) a(s)+b(-s) b(s)=d(-s) d(s) .
$$

Step 2: (Matrix construction) Construct

$$
\boldsymbol{H}=\boldsymbol{J} \boldsymbol{L}_{d}^{-1} \boldsymbol{J}\left[\boldsymbol{L}_{b} \boldsymbol{J},-\boldsymbol{L}_{a} \boldsymbol{J}\right]\left[\begin{array}{ll}
\boldsymbol{L}_{a} & \boldsymbol{L}_{b} \\
\boldsymbol{U}_{a} & \boldsymbol{U}_{b}
\end{array}\right]^{-1}\left[\begin{array}{l}
\boldsymbol{L}_{d} \\
\boldsymbol{U}_{d}
\end{array}\right]
$$

Step 3: (Eigen-computation) Find the eigenvalue of $\boldsymbol{H}$ whose magnitude equals to the spectral radius $\rho(\boldsymbol{H})$. Let $\boldsymbol{e}$ be an eigenvector corresponding to this eigenvalue ${ }^{1}$.
Step 4: (Pole placement) Compute

$$
\left[\begin{array}{c}
\boldsymbol{p} \\
\boldsymbol{q}
\end{array}\right]=\left[\begin{array}{cc}
\boldsymbol{L}_{a} & \boldsymbol{L}_{b} \\
\boldsymbol{U}_{a} & \boldsymbol{U}_{b}
\end{array}\right]^{-1}\left[\begin{array}{c}
\boldsymbol{L}_{d} \\
\boldsymbol{U}_{d}
\end{array}\right] \boldsymbol{e} \quad \text { and } \quad\left\{\begin{array}{l}
p(s)=\left[\begin{array}{llll}
s^{n-1} & s^{n-2} & \cdots & 1
\end{array}\right] \boldsymbol{p} \\
q(s)=\left[\begin{array}{llll}
s^{n-1} & s^{n-2} & \cdots & 1
\end{array}\right] \boldsymbol{q}
\end{array} .\right.
$$

An optimal controller is given by $C_{\text {opt }}(s)=\frac{q(s)}{p(s)}$.
Step 5: (Optimal robustness margin computation)

$$
\alpha(P(s))=\frac{1}{\sqrt{1+\rho^{2}(\boldsymbol{H})}}
$$

Notice that only basic matricial and polynomial manipulations, such as spectral factorization, eigenvalue decomposition and matrix inversion are required in the algorithm above. See Section 3.2 for an illustrative example of applying the algorithm.

Whereas Steps 2, 3, and 5 in Algorithm 1 are concerned with the optimal control design, Steps 1 and 4 are standard and well known, as we elaborate below. Denote by $\mathscr{P}_{n}$ the set of all the polynomials with real coefficients and of degree $n$. That is, for $d(s) \in \mathscr{P}_{n}$, it holds $d(s)=d_{0} s^{n}+d_{1} s^{n-1}+\cdots+d_{n}$ with $d_{0} \neq 0$. This polynomial $d(s)$ is said to be stable if all its roots have negative real parts.

Let the plant $P(s)$ be given as in (5). Observe that the polynomial

$$
\begin{equation*}
a(-s) a(s)+b(-s) b(s) \tag{6}
\end{equation*}
$$

is self-conjugate, i.e., its conjugate coincides with itself. Consequently, if $z$ is a root of this polynomial, then so is $-z$. Together with the coprimeness of $a(s)$ and $b(s)$,

[^1]it follows that this polynomial has no roots on the imaginary axis and all its roots are symmetric about the imaginary axis. Step 1 in Algorithm 1 can be carried out by first solving for the roots of the polynomial in (6) and then obtaining a stable polynomial $d(s) \in \mathscr{P}_{n}$ such that
$$
\begin{equation*}
a(-s) a(s)+b(-s) b(s)=d(-s) d(s) \tag{7}
\end{equation*}
$$

This process is known as the spectral factorization [Kailath, 1980, Section 3.4], [Qiu and Zhou, 2009, Section 8.1].

Given two coprime polynomials with real coefficients $p(s)$ and $q(s)$, by defining

$$
\begin{equation*}
C(s):=\frac{q(s)}{p(s)} \tag{8}
\end{equation*}
$$

we know from the definition of the Gang of Four transfer matrix $P(s) \# C(s)$ that the closed-loop poles are the roots of the characteristic polynomial

$$
a(s) p(s)+b(s) q(s)
$$

One way to obtain $p(s)$ and $q(s)$ is via the pole placement method as follows. Let $e(s) \in \mathscr{P}_{n-1}$ be a stable polynomial. By solving the following polynomial Diophantine equation [Kailath, 1980, Section 4.5] [Qiu and Zhou, 2009, Section 3.6]

$$
\begin{equation*}
a(s) p(s)+b(s) q(s)=d(s) e(s) \tag{9}
\end{equation*}
$$

we obtain $p(s)$ and $q(s)$ with $\max \{\operatorname{deg} p(s), \operatorname{deg} q(s)\} \leq n-1$. A controller $C(s)$ defined as in (8) then places the closed-loop poles at the roots of $d(s) e(s)$. Such a process is called the pole placement design, and the resulting $C(s)$ is called a pole placement controller. In particular, equating the coefficients in (9) yields the following system of linear equations:

$$
\left[\begin{array}{cc}
\boldsymbol{L}_{a} & \boldsymbol{L}_{b}  \tag{10}\\
\boldsymbol{U}_{a} & \boldsymbol{U}_{b}
\end{array}\right]\left[\begin{array}{c}
\boldsymbol{p} \\
\boldsymbol{q}
\end{array}\right]=\left[\begin{array}{c}
\boldsymbol{L}_{d} \\
\boldsymbol{U}_{d}
\end{array}\right] \boldsymbol{e},
$$

where the elements in $\boldsymbol{p}$ and $\boldsymbol{q}$ are the unknowns. The matrix

$$
\left[\begin{array}{cc}
\boldsymbol{L}_{a} & \boldsymbol{L}_{b} \\
\boldsymbol{U}_{a} & \boldsymbol{U}_{b}
\end{array}\right]
$$

is called a Sylvester's resultant matrix [Qiu and Zhou, 2009, Section 3.6], which is a $2 n$-by- $2 n$ nonsingular matrix as $a(s)$ and $b(s)$ are coprime. By inverting this matrix as in Step 4 of Algorithm 1, we obtain the solution to equation (9), as well as the pole placement controller.

It is now obvious that Steps 2 and 3 of Algorithm 1 serve the purpose of computing a partial set of the closed-loop poles, based on which the pole placement controller resulting from Step 4 gives rise to an optimally robust controller. The proof of this fact is deferred to Section 4.

### 3.2 An Illustrative Example

Here we revisit the simple example of a double integrator and apply Algorithm 1 to obtain an optimally robust controller.

Example 1. Let

$$
P(s)=\frac{1}{s^{2}}
$$

Objective: find an optimal controller $C(s)$ such that $\|P(s) \# C(s)\|_{\infty}$ is minimized with Algorithm 1.

1. (Spectral factorization)

$$
s^{4}+1=d(-s) d(s)
$$

This gives $d(s)=s^{2}+\sqrt{2} s+1$.
2. (Matrix computation) We can compute that

$$
\boldsymbol{H}=\left[\begin{array}{cc}
1 & \sqrt{2} \\
\sqrt{2} & 1
\end{array}\right]
$$

3. (Eigen-computation) The eigenvalues of $\boldsymbol{H}$ are $1 \pm \sqrt{2}$. The eigenvalue with the largest magnitude is $1+\sqrt{2}$ and the corresponding eigenvector satisfies

$$
\left[\begin{array}{cc}
1 & \sqrt{2} \\
\sqrt{2} & 1
\end{array}\right] \boldsymbol{e}=(1+\sqrt{2}) \boldsymbol{e}
$$

This gives $\boldsymbol{e}=\left[\begin{array}{l}1 \\ 1\end{array}\right]$. Thus,

$$
e(s)=\left[\begin{array}{ll}
s & 1
\end{array}\right] \boldsymbol{e}=s+1
$$

4. (Pole placement) We obtain $p(s)=s+1+\sqrt{2}$ and $q(s)=(1+\sqrt{2}) s+1$ from

$$
\left[\begin{array}{l}
\boldsymbol{p} \\
\boldsymbol{q}
\end{array}\right]=\left[\begin{array}{ll}
\boldsymbol{L}_{a} & \boldsymbol{L}_{b} \\
\boldsymbol{U}_{a} & \boldsymbol{U}_{b}
\end{array}\right]^{-1}\left[\begin{array}{l}
\boldsymbol{L}_{d} \\
\boldsymbol{U}_{d}
\end{array}\right] \boldsymbol{e}=\left[\begin{array}{llll}
1 & 0 & 0 & 0 \\
0 & 1 & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{array}\right]^{-1}\left[\begin{array}{cc}
1 & 0 \\
\sqrt{2} & 1 \\
1 & \sqrt{2} \\
0 & 1
\end{array}\right]\left[\begin{array}{l}
1 \\
1
\end{array}\right]=\left[\begin{array}{c}
1 \\
1+\sqrt{2} \\
1+\sqrt{2} \\
1
\end{array}\right]
$$

An optimally robust controller is then given by

$$
C_{\mathrm{opt}}(s)=\frac{(1+\sqrt{2}) s+1}{s+1+\sqrt{2}}
$$

5. (Optimal robustness margin computation)

$$
\alpha(P(s))=\frac{1}{\sqrt{4+2 \sqrt{2}}}
$$

### 3.3 The Nongeneric Case

In Step 3 of Algorithm 1, the generic case where $\boldsymbol{H}$ admits a unique eigenvalue of magnitude $\rho(\boldsymbol{H})$ is dealt with. Here we mention without proof a method to handle the singular case where $\boldsymbol{H}$ has multiple eigenvalues of magnitude $\rho(\boldsymbol{H})$. This will not be pursued further elsewhere in this chapter. First we introduce some notation. For a square matrix $\boldsymbol{A} \in \mathbb{R}^{n \times n}$, denote by $\lambda_{k}(\boldsymbol{A}), k=1,2, \ldots, n$ its $k$-th eigenvalue counting multiplicity, ordered according to

$$
\left|\lambda_{1}(\boldsymbol{A})\right|=\cdots=\left|\lambda_{v}(\boldsymbol{A})\right|>\left|\lambda_{v+1}(\boldsymbol{A})\right| \geq \cdots \geq\left|\lambda_{n}(\boldsymbol{A})\right| .
$$

The spectral radius of $\boldsymbol{H}, \rho(\boldsymbol{H})$, is hence $\left|\lambda_{1}(\boldsymbol{H})\right|$. Let the number of the eigenvalues of magnitude $\rho(\boldsymbol{H})$ be $m(\boldsymbol{H}):=v>1$.

It can be shown using the spectral factorization relation in Step 1 of Algorithm 1 that $\boldsymbol{H}$ is diagonalizable and all its eigenvalues are real, hence either or both of $\rho(\boldsymbol{H})$ and $-\rho(\boldsymbol{H})$ are eigenvalues of $\boldsymbol{H}$. If $\rho(\boldsymbol{H})$ is an eigenvalue, let $\mathscr{E}_{1}$ be the corresponding eigenspace; otherwise $\mathscr{E}_{1}=\{\mathbf{0}\}$. Similarly, let $\mathscr{E}_{2}$ be the eigenspace corresponding to $-\rho(\boldsymbol{H})$. Then Algorithm 1 with Step 3 replaced by Step 3* below yields an optimally robust controller whose order is no larger than $n-m(\boldsymbol{H})$ :

Step 3*: (Eigen-computation) Find $\mathbf{0} \neq \boldsymbol{e} \in \mathscr{E}_{1} \cup \mathscr{E}_{2}$ such that the degree of $e(s)$ is minimized, where

$$
e(s)=\left[\begin{array}{llll}
s^{n-1} & s^{n-2} & \cdots & 1
\end{array}\right] \boldsymbol{e}
$$

The following example of a special all-pass system illustrates how we utilize the algorithm when $m(\boldsymbol{H})>1$.

Example 2. Consider the following all-pass plant

$$
P(s)=\frac{(s-1)(s-2)(s-3)}{(s+1)(s+2)(s+3)}=\frac{s^{3}-6 s^{2}+11 s-6}{s^{3}+6 s^{2}+11 s+6} .
$$

Objective: find an optimal controller $C(s)$ such that $\|P(s) \# C(s)\|_{\infty}$ is minimized using Algorithm 1 equipped with Step 3* above.

1. (Spectral factorization)

$$
d(s)=\sqrt{2}(s+1)(s+2)(s+3)=\sqrt{2}\left(s^{3}+6 s^{2}+11 s+6\right)
$$

2. (Matrix computation) We can compute that

$$
\boldsymbol{H}=\left[\begin{array}{ccc}
-1 & 0 & 0 \\
0 & 1 & 0 \\
0 & 0 & -1
\end{array}\right]
$$

3. (Eigen-computation) The eigenvalues of $\boldsymbol{H}$ are $1,-1$ and -1 , all of which have magnitude 1 . Hence $m(\boldsymbol{H})=n=3$. The eigenspaces corresponding to eigenvalues 1 and -1 are, respectively,

$$
\mathscr{E}_{1}=\operatorname{span}\left[\begin{array}{l}
0 \\
1 \\
0
\end{array}\right] \quad \text { and } \quad \mathscr{E}_{2}=\operatorname{span}\left[\begin{array}{ll}
1 & 0 \\
0 & 0 \\
0 & 1
\end{array}\right]
$$

The vector $\mathbf{0} \neq \boldsymbol{e} \in \mathscr{E}_{1} \cup \mathscr{E}_{2}$ such that the degree of $e(s)$ is minimized is given by

$$
\boldsymbol{e}=\left[\begin{array}{l}
0 \\
0 \\
1
\end{array}\right], \quad \text { whereby } e(s)=1, \quad \text { and } \quad \operatorname{deg} e(s)=0=n-m(\boldsymbol{H})
$$

4. (Pole placement) We obtain $p(s)=\sqrt{2}$ and $q(s)=0$ from

$$
[\boldsymbol{p}]=\left[\begin{array}{cccccc}
1 & 0 & 0 & 1 & 0 & 0 \\
6 & 1 & 0 & -6 & 1 & 0 \\
11 & 6 & 1 & 11 & -6 & 1 \\
6 & 11 & 6 & -6 & 11 & -6 \\
0 & 6 & 11 & 0 & -6 & 11 \\
0 & 0 & 6 & 0 & 0 & -6
\end{array}\right]^{-1}\left[\begin{array}{ccc}
\sqrt{2} & 0 & 0 \\
6 \sqrt{2} & \sqrt{2} & 0 \\
11 \sqrt{2} & 6 \sqrt{2} & \sqrt{2} \\
6 \sqrt{2} & 11 \sqrt{2} & 6 \sqrt{2} \\
0 & 6 \sqrt{2} & 11 \sqrt{2} \\
0 & 0 & 6 \sqrt{2}
\end{array}\right]\left[\begin{array}{l}
0 \\
0 \\
1
\end{array}\right]=\left[\begin{array}{c}
0 \\
0 \\
\sqrt{2} \\
0 \\
0 \\
0
\end{array}\right]
$$

Hence, an optimally robust controller is given by

$$
C_{\mathrm{opt}}(s)=0
$$

5. (Optimal robustness margin computation)

$$
\alpha(P(s))=\frac{1}{\sqrt{2}}
$$

## 4 Proof of Optimality

The purpose of this section is to prove that the controller $C_{\text {opt }}(s)$ obtained from Algorithm 1 is optimal, in the sense that it satisfies

$$
C_{\mathrm{opt}}(s)=\underset{C(s)}{\arg \min }\|P(s) \# C(s)\|_{\infty}
$$

### 4.1 Preliminaries

Given a plant $P(s)=\frac{b(s)}{a(s)}$ and a stable polynomial $e(s)$, we can rewrite the spectral factorization in (7) and the pole placement in (9) as, respectively,

$$
\begin{equation*}
M(-s) M(s)+N(-s) N(s)=1 \quad \text { and } \quad M(s) X(s)+N(s) Y(s)=1 \tag{11}
\end{equation*}
$$

where

$$
\begin{equation*}
M(s):=\frac{a(s)}{d(s)}, \quad N(s):=\frac{b(s)}{d(s)}, \quad X(s):=\frac{p(s)}{e(s)}, \quad \text { and } \quad Y(s):=\frac{q(s)}{e(s)} \tag{12}
\end{equation*}
$$

Based on these relations, the set of all controllers $C(s)$ for which $P(s) \# C(s)$ is stable is given by the Youla parametrization [Youla et al., 1976] as

$$
\begin{equation*}
\mathscr{S}(P(s))=\left\{C(s)=\frac{Y(s)+M(s) Q(s)}{X(s)-N(s) Q(s)}: Q(s) \in \mathscr{R} \mathscr{H}_{\infty}\right\} \tag{13}
\end{equation*}
$$

Obviously, an optimally robust controller belongs to the set $\mathscr{S}(P(s))$.
It can be shown with some algebraic manipulations [Qiu and Zhou, 2009, Chapter 9] that

$$
\|P(s) \# C(s)\|_{\infty}=\left(1+\left\|\frac{P(-s)-C(s)}{1+P(s) C(s)}\right\|_{\infty}^{2}\right)^{\frac{1}{2}}
$$

As a consequence,

$$
\min _{C(s) \in \mathscr{S}(P(s))}\|P(s) \# C(s)\|_{\infty}
$$

is equivalent to

$$
\begin{equation*}
\min _{C(s) \in \mathscr{S}(P(s))}\left\|\frac{P(-s)-C(s)}{1+P(s) C(s)}\right\|_{\infty}=: \gamma(P(s)) \tag{14}
\end{equation*}
$$

Furthermore, the optimal robust stability margin is

$$
\alpha(P(s))=\frac{1}{\sqrt{1+\gamma^{2}(P(s))}}
$$

Later in the section, we will show that $\gamma(P(s))=\rho(\boldsymbol{H})$, whereby

$$
\alpha(P(s))=\frac{1}{\sqrt{1+\rho^{2}(\boldsymbol{H})}}
$$

as in Step 5 of Algorithm 1.
In the sequel, we derive an alternative form, which is easier to work with, for the $\mathscr{H}_{\infty}$ optimal control problem in (14). In particular, given the set of all stabilizing controllers $\mathscr{S}(P(s))$ in (13) and $M(s), N(s), X(s), Y(s)$ defined in (12), we have

$$
\begin{align*}
\gamma(P(s)) & =\inf _{C(s) \in \mathscr{S}(P(s))}\left\|\frac{P(-s)-C(s)}{1+P(s) C(s)}\right\|_{\infty} \\
& =\inf _{Q(s) \in \mathscr{R} \mathscr{H}}\left\|\frac{M(s)[N(-s) X(s)-M(-s) Y(s)-Q(s)]}{M(-s)}\right\|_{\infty} \\
& =\inf _{Q(s) \in \mathscr{R} \mathscr{H}}\|N(-s) X(s)-M(-s) Y(s)-Q(s)\|_{\infty} \\
& =\inf _{Q(s) \in \mathscr{R} \mathscr{H}}\|G(s)-Q(s)\|_{\infty} \tag{15}
\end{align*}
$$

where

$$
\begin{equation*}
G(s):=N(-s) X(s)-M(-s) Y(s)=\frac{b(-s) p(s)-a(-s) q(s)}{d(-s) e(s)} \in \mathscr{R} \mathscr{L}_{\infty} \tag{16}
\end{equation*}
$$

and the third equality follows from the fact that

$$
\frac{M(s)}{M(-s)}
$$

is an all-pass transfer function. Consequently, solving for an optimally robust controller is equivalent to finding a $Q(s) \in \mathscr{R} \mathscr{H}{ }_{\infty}$ that lies the closest to $G(s) \in \mathscr{R} \mathscr{L}_{\infty}$ in the $\mathscr{L}_{\infty}$ norm. This special $\mathscr{H}_{\infty}$ optimal control problem is a Nehari's problem [Nehari, 1957], [Fuhrmann, 2012, Chapter 12], which is closely related to the partial pole placement problem introduced in what follows.

### 4.2 Partial Pole Placement

Definition 1. Given an $n$th order plant $P(s)=\frac{b(s)}{a(s)}$ and a stable polynomial $d(s) \in \mathscr{P}_{n}$ obtained from the spectral factorization in (7), we say that a triplet of polynomials $\{p(s), q(s), e(s)\}$ solves the partial pole placement problem for $\lambda \in \mathbb{R}$ if it satisfies

$$
\begin{align*}
a(s) p(s)+b(s) q(s) & =d(s) e(s) \\
b(-s) p(s)-a(-s) q(s) & =\lambda d(s) e(-s)  \tag{17}\\
\max \{\operatorname{deg} p(s), \operatorname{deg} q(s)\} & \leq \operatorname{deg} e(s) \leq n-1
\end{align*}
$$

A partial pole placement problem is distinguished from a pole placement problem in (9), since the closed-loop poles, namely the roots of $d(s) e(s)$, are not completely prescribed ahead of time and need to be determined from the equations in (17). Two questions arise naturally from this problem:
(i) What are the possible solutions to the partial pole placement problem?
(ii) How are these solutions related to the Nehari's problem in (15)?

We answer these questions below, and in doing so complete the main part of the derivation for the optimal controller from Algorithm 1. We begin with Question (ii).

Recall the expression of $G(s)$ in (16). In a similar manner, define a series of transfer functions in $\mathscr{R} \mathscr{L}_{\infty}$ for $k=1,2, \ldots, n$ by

$$
\begin{equation*}
G_{k}(s)=\frac{b(-s) p_{k}(s)-a(-s) q_{k}(s)}{d(-s) e_{k}(s)} \tag{18}
\end{equation*}
$$

where $\left\{p_{k}(s), q_{k}(s), e_{k}(s)\right\}$ is a solution of (17) with respect to $\lambda_{k}$ and $e_{k}(s)$ has exactly $k-1$ anti-stable roots. In particular, $e_{1}(s)$ is stable. Recall that the $e(s) \in \mathscr{P}_{n-1}$ in (16) is required to be a stable polynomial. Henceforth, let $e(s)=e_{1}(s)$, whereby $G(s)=G_{1}(s)$. Denote by $\mathscr{R} \mathscr{L}_{\infty}^{[k]} \subset \mathscr{R} \mathscr{L}_{\infty}$ the set of all the transfer functions that have at most $k-1$ anti-stable poles. Specifically, $\mathscr{R} \mathscr{L}_{\infty}^{[1]}=\mathscr{R} \mathscr{H}_{\infty}$. Similarly to (15), consider the series of optimization problems

$$
\begin{equation*}
\inf _{Q_{k}(s) \in \mathscr{R} \mathscr{L}_{\infty}^{[k]}}\left\|G_{k}(s)-Q_{k}(s)\right\|_{\infty}, k=1,2, \ldots, n \tag{19}
\end{equation*}
$$

When $k=1$, the optimization problem reduces to (15). As $k$ increases, the enlargement of the feasible set of the $k$ th optimization problem is more than enough to compensate for the additional anti-stable pole in $G_{k}(s)$. As a result, for $k= 1,2, \ldots, n-1$, we have

$$
\begin{equation*}
\inf _{Q_{k}(s) \in \mathscr{R} \mathscr{L}_{\infty}^{[k]}}\left\|G_{k}(s)-Q_{k}(s)\right\|_{\infty} \geq \inf _{Q_{k+1}(s) \in \mathscr{R} \mathscr{L}_{\infty}^{[k+1]}}\left\|G_{k+1}(s)-Q_{k+1}(s)\right\|_{\infty} . \tag{20}
\end{equation*}
$$

The following lemma shows that the series of optimization problems above admit analytic solutions.

Lemma 1. Given $G_{k}(s)$ as defined in (18), we have

$$
\inf _{Q_{k}(s) \in \mathscr{R} \mathscr{L}_{\infty}^{[k]}}\left\|G_{k}(s)-Q_{k}(s)\right\|_{\infty}=\left|\lambda_{k}\right|
$$

where the infimum is achieved when $Q_{k}(s)=0$.
Proof. Since $\left\{p_{k}(s), q_{k}(s), e_{k}(s)\right\}$ is a solution of (17) with respect to $\lambda_{k}$, it follows that

$$
G_{k}(s)=\frac{b(-s) p_{k}(s)-a(-s) q_{k}(s)}{d(-s) e_{k}(s)}=\lambda_{k} \frac{d(s) e_{k}(-s)}{d(-s) e_{k}(s)} .
$$

Consequently,

$$
\begin{equation*}
\inf _{Q_{k}(s) \in \mathscr{R} \mathscr{L}_{\infty}^{[k]}}\left\|G_{k}(s)-Q_{k}(s)\right\|_{\infty} \leq\left\|G_{k}(s)-0\right\|_{\infty}=\left\|\lambda_{k} \frac{d(s) e_{k}(-s)}{d(-s) e_{k}(s)}\right\|_{\infty}=\left|\lambda_{k}\right|, \tag{21}
\end{equation*}
$$

where the fact that

$$
\frac{d(s) e_{k}(-s)}{d(-s) e_{k}(s)}
$$

is all-pass has been used. We show below

$$
\inf _{Q_{k}(s) \in \mathscr{R} \mathscr{L}_{\infty}^{[k]}}\left\|G_{k}(s)-Q_{k}(s)\right\|_{\infty} \geq\left|\lambda_{k}\right|
$$

from which it follows that equality is achieved when $Q_{k}(s)=0$.
Let $e_{k}(s)=f_{k}(s) g_{k}(-s)$, where $f_{k}(s)$ and $g_{k}(s)$ are stable polynomials and $\operatorname{deg} g_{k}(s)=k-1$. For an arbitrary transfer function $Q_{k}(s) \in \mathscr{R} \mathscr{L}_{\infty}^{[k]}$, we can write

$$
Q_{k}(s)=\frac{h_{k}(s)}{h_{k}(-s)} \tilde{Q}_{k}(s)
$$

where $h_{k}(s) \in \mathscr{P}_{k-1}$ is stable and $\tilde{Q}_{k}(s) \in \mathscr{R} \mathscr{H}_{\infty}$. Define

$$
U_{k}(s):=\frac{f_{k}(s) h_{k}(-s)}{d(s)}
$$

which is an element in $\mathscr{H}_{2}$ since $d(s)$ is stable and

$$
\operatorname{deg} f_{k}(s)+\operatorname{deg} h_{k}(s) \leq \operatorname{deg} e_{k}(s)<\operatorname{deg} d(s)
$$

Observe that

$$
G_{k}(s) U_{k}(s)=\lambda_{k} \frac{h_{k}(-s) f_{k}(-s) g_{k}(s)}{d(-s) g_{k}(-s)} \in \mathscr{R} \mathscr{H}_{2}^{\perp}
$$

and

$$
\begin{aligned}
& \left\|G_{k}(s) U_{k}(s)\right\|_{2}= \\
& \quad\left|\lambda_{k}\right|\left\|\frac{h_{k}(-s) f_{k}(-s) g_{k}(s)}{d(-s) g_{k}(-s)}\right\|_{2}=\left|\lambda_{k}\right|\left\|\frac{h_{k}(-s) f_{k}(-s)}{d(-s)}\right\|_{2}=\left|\lambda_{k}\right|\left\|U_{k}(s)\right\|_{2}
\end{aligned}
$$

On the other hand, we have

$$
Q_{k}(s) U_{k}(s)=\frac{f_{k}(s) h_{k}(s)}{d(s)} \tilde{Q}_{k}(s) \in \mathscr{H}_{2}
$$

Therefore,

$$
\begin{aligned}
\left\|G_{k}(s)-Q_{k}(s)\right\|_{\infty} & \geq \frac{\left\|G_{k}(s) U_{k}(s)-Q_{k}(s) U_{k}(s)\right\|_{2}}{\left\|U_{k}(s)\right\|_{2}} \\
& =\sqrt{\frac{\left\|G_{k}(s) U_{k}(s)\right\|_{2}^{2}}{\left\|U_{k}(s)\right\|_{2}^{2}}+\frac{\left\|Q_{k}(s) U_{k}(s)\right\|_{2}^{2}}{\left\|U_{k}(s)\right\|_{2}^{2}}} \\
& =\sqrt{\left|\lambda_{k}\right|^{2}+\frac{\left\|Q_{k}(s) U_{k}(s)\right\|_{2}^{2}}{\left\|U_{k}(s)\right\|_{2}^{2}}} \\
& \geq\left|\lambda_{k}\right|
\end{aligned}
$$

as required.

Below we provide an answer to Question (i). Lying at the core of the answer is the matrix $\boldsymbol{H}$ defined in Step 2 of Algorithm 1. First recall the notation introduced at the start of Section 3.1.

Lemma 2. A triplet of polynomials $\{p(s), q(s), e(s)\}$ is a solution to the partial pole placement problem in Definition 1 with respect to a $\lambda \in \mathbb{R}$ if, and only if, $\{\boldsymbol{p}, \boldsymbol{q}, \boldsymbol{e}\}$ satisfies

$$
\boldsymbol{H} \boldsymbol{e}=(-1)^{n} \lambda \boldsymbol{e}
$$

and

$$
\left[\begin{array}{l}
\boldsymbol{p}  \tag{22}\\
\boldsymbol{q}
\end{array}\right]=\left[\begin{array}{cc}
\boldsymbol{L}_{a} & \boldsymbol{L}_{b} \\
\boldsymbol{U}_{a} & \boldsymbol{U}_{b}
\end{array}\right]^{-1}\left[\begin{array}{c}
\boldsymbol{L}_{d} \\
\boldsymbol{U}_{d}
\end{array}\right] \boldsymbol{e}
$$

where

$$
\boldsymbol{H}:=\boldsymbol{J} \boldsymbol{L}_{d}^{-1} \boldsymbol{J}\left[\boldsymbol{L}_{b} \boldsymbol{J}-\boldsymbol{L}_{a} \boldsymbol{J}\right]\left[\begin{array}{cc}
\boldsymbol{L}_{a} & \boldsymbol{L}_{b} \\
\boldsymbol{U}_{a} & \boldsymbol{U}_{b}
\end{array}\right]^{-1}\left[\begin{array}{c}
\boldsymbol{L}_{d} \\
\boldsymbol{U}_{d}
\end{array}\right]
$$

Proof. Equating the coefficients of the polynomials on both sides of the equations in (17) results in the following two systems of linear equations:

$$
\begin{align*}
{\left[\begin{array}{ll}
\boldsymbol{L}_{a} & \boldsymbol{L}_{b} \\
\boldsymbol{U}_{a} & \boldsymbol{U}_{b}
\end{array}\right]\left[\begin{array}{l}
\boldsymbol{p} \\
\boldsymbol{q}
\end{array}\right] } & =\left[\begin{array}{c}
\boldsymbol{L}_{d} \\
\boldsymbol{U}_{d}
\end{array}\right] \boldsymbol{e}, \\
{\left[\begin{array}{ll}
(-1)^{n} \boldsymbol{J} \\
J
\end{array}\right]\left[\begin{array}{ll}
\boldsymbol{L}_{b} & -\boldsymbol{L}_{a} \\
\boldsymbol{U}_{b} & -\boldsymbol{U}_{a}
\end{array}\right]\left[\begin{array}{l}
\boldsymbol{J} \\
\boldsymbol{J}
\end{array}\right]\left[\begin{array}{l}
\boldsymbol{p} \\
\boldsymbol{q}
\end{array}\right] } & =\lambda\left[\begin{array}{c}
\boldsymbol{L}_{d} \\
\boldsymbol{U}_{d}
\end{array}\right] \boldsymbol{J e} . \tag{23}
\end{align*}
$$

Since $a(s)$ and $b(s)$ are coprime, the matrix

$$
\left[\begin{array}{cc}
\boldsymbol{L}_{a} & \boldsymbol{L}_{b} \\
\boldsymbol{U}_{a} & \boldsymbol{U}_{b}
\end{array}\right]
$$

is invertible. Thus, from (23), we obtain (22) and

$$
\left[\begin{array}{ll}
\boldsymbol{J} &  \tag{24}\\
& (-1)^{n} \boldsymbol{J}
\end{array}\right]\left[\begin{array}{ll}
\boldsymbol{L}_{b} & -\boldsymbol{L}_{a} \\
\boldsymbol{U}_{b} & -\boldsymbol{U}_{a}
\end{array}\right]\left[\begin{array}{ll}
\boldsymbol{J} & \\
& \boldsymbol{J}
\end{array}\right]\left[\begin{array}{ll}
\boldsymbol{L}_{a} & \boldsymbol{L}_{b} \\
\boldsymbol{U}_{a} & \boldsymbol{U}_{b}
\end{array}\right]^{-1}\left[\begin{array}{l}
\boldsymbol{L}_{d} \\
\boldsymbol{U}_{d}
\end{array}\right] \boldsymbol{e}=(-1)^{n} \lambda\left[\begin{array}{l}
\boldsymbol{L}_{d} \\
\boldsymbol{U}_{d}
\end{array}\right] \boldsymbol{J} \boldsymbol{e}
$$

There are a total of $2 n$ linear equations involving the elements of $\boldsymbol{e}$ in (24). By equating the coefficients in the spectral factorization in (7), one may show that the first $n$ equations are identical to the last $n$ ones after some algebraic manipulations. Hence it suffices to consider the first $n$ rows of (24):

$$
\boldsymbol{J}\left[\boldsymbol{L}_{b} \boldsymbol{J}-\boldsymbol{L}_{a} \boldsymbol{J}\right]\left[\begin{array}{cc}
\boldsymbol{L}_{a} & \boldsymbol{L}_{b} \\
\boldsymbol{U}_{a} & \boldsymbol{U}_{b}
\end{array}\right]^{-1}\left[\begin{array}{c}
\boldsymbol{L}_{d} \\
\boldsymbol{U}_{d}
\end{array}\right] \boldsymbol{e}=(-1)^{n} \lambda \boldsymbol{L}_{d} \boldsymbol{J} \boldsymbol{e}
$$

Since $\operatorname{deg} d(s)=n$, or equivalently, $d_{0} \neq 0, \boldsymbol{L}_{d}$ is invertible. Left-multiplying both sides of the equation above by $\boldsymbol{J} \boldsymbol{L}_{d}^{-1}$ yields

$$
\boldsymbol{H} \boldsymbol{e}=(-1)^{n} \lambda \boldsymbol{e}
$$

In other words, $\left\{(-1)^{n} \lambda, \boldsymbol{e}\right\}$ is an eigenpair of $\boldsymbol{H}$.

### 4.3 Optimally Robust Controller

Based on the previous development, we state the main result as follows.
Theorem 2. The controller $C_{\mathrm{opt}}(s)$ defined in Algorithm 1 is optimally robust, in the sense that

$$
C_{\mathrm{opt}}(s)=\underset{C(s) \in \mathscr{S}(P(s))}{\arg \min }\left\|\frac{P(-s)-C(s)}{1+P(s) C(s)}\right\|_{\infty}=\underset{C(s) \in \mathscr{S}(P(s))}{\arg \min }\|P(s) \# C(s)\|_{\infty}
$$

Moreover, the optimal robustness margin is

$$
\alpha(P(s))=\frac{1}{\sqrt{1+\rho^{2}(\boldsymbol{H})}}
$$

Proof. By Lemma 1 and (20), we have for $k=1,2, \ldots, n-1$,

$$
\begin{aligned}
\left|\lambda_{k}\right|=\left\|G_{k}(s)\right\|_{\infty} & =\inf _{Q_{k}(s) \in \mathscr{R} \mathscr{L}_{\infty}^{[k]}}\left\|G_{k}(s)-Q_{k}(s)\right\|_{\infty} \\
& \geq \inf _{Q_{k+1}(s) \in \mathscr{R} \mathscr{L}_{\infty}^{[k+1]}}\left\|G_{k+1}(s)-Q_{k+1}(s)\right\|_{\infty}=\left\|G_{k+1}(s)\right\|_{\infty}=\left|\lambda_{k+1}\right|,
\end{aligned}
$$

where

$$
G_{k}(s):=\frac{b(-s) p_{k}(s)-a(-s) q_{k}(s)}{d(-s) e_{k}(s)}
$$

is as defined in (18), $\left\{p_{k}(s), q_{k}(s), e_{k}(s)\right\}$ is a solution to the partial pole placement problem in Definition 15 with respect to $\lambda_{k}$ and $e_{k}(s)$ has exactly $k-1$ anti-stable roots. Furthermore, by Lemma 2, each $\left|\lambda_{k}\right|$ is the magnitude of an eigenvalue of $\boldsymbol{H}$ defined in Step 2 of Algorithm 1. In particular, $\left|\lambda_{1}\right|=\rho(\boldsymbol{H})$. Since $e_{1}(s)$ is stable, it follows from (15) that

$$
\begin{aligned}
\gamma(P(s)) & =\inf _{C(s) \in \mathscr{S}(P(s))}\left\|\frac{P(-s)-C(s)}{1+P(s) C(s)}\right\|_{\infty} \\
& =\inf _{Q(s) \in \mathscr{R} \mathscr{H}}\left\|G_{1}(s)-Q(s)\right\|_{\infty} \\
& =\inf _{Q_{1}(s) \in \mathscr{R} \mathscr{L}_{\infty}^{[1]}}\left\|G_{1}(s)-Q_{1}(s)\right\|_{\infty} \\
& =\left\|G_{1}(s)\right\|_{\infty} \\
& =\left|\lambda_{1}\right|=\rho(\boldsymbol{H})
\end{aligned}
$$

Therefore,

$$
\alpha(P(s))=\frac{1}{\sqrt{1+\gamma^{2}(P(s))}}=\frac{1}{\sqrt{1+\rho^{2}(\boldsymbol{H})}},
$$

and again by Lemma 2, Steps 3 and 4 of Algorithm 1 yield a triplet of polynomials $\{p(s), q(s), e(s)\}$ satisfying $p(s)=p_{1}(s), q(s)=q_{1}(s)$, and $e(s)=e_{1}(s)$, whereby

$$
G_{1}(s):=\frac{b(-s) p_{1}(s)-a(-s) q_{1}(s)}{d(-s) e_{1}(s)}=\frac{b(-s) p(s)-a(-s) q(s)}{d(-s) e(s)}
$$

By defining $C_{\text {opt }}(s):=\frac{q(s)}{p(s)}$, we have

$$
\begin{aligned}
\left\|G_{1}(s)\right\|_{\infty} & =\left\|\frac{b(-s) p(s)-a(-s) q(s)}{d(-s) e(s)}\right\|_{\infty} \\
& =\left\|\frac{d(-s) a(s)}{a(-s) d(s)} \frac{b(-s) p(s)-a(-s) q(s)}{d(-s) e(s)}\right\|_{\infty} \\
& =\left\|\frac{P(-s)-C_{\mathrm{opt}}(s)}{1+P(s) C_{\mathrm{opt}}(s)}\right\|_{\infty}
\end{aligned}
$$

where the first equality in (17) has been used. In other words, the $C_{\text {opt }}(s)$ obtained in Algorithm 1 is optimally robust.

## 5 Case Study: Control of a USUAL Inverted Pendulum

The inverted pendulum system has been one of the most popular control education apparatus since the 1950s. The system has been widely utilized for verifying the effectiveness of stabilizing algorithms due to its unstable and under-actuated properties. The system mimics the human stick balancing game: balancing a long stick upward on our finger tip. In the game, our fingers move in a horizontal plane and the stick can fall in all directions. In this scenario, the state of the stick is observed directly by the human vision. Unlike the game, the inverted pendulum in this case study, whose cart moves linearly along a straight rail and rod can only fail either to the front or to the back of the cart, is a simplified one-dimensional version of the game. See Fig. 5 for an illustration.

![](https://cdn.mathpix.com/cropped/e749505a-107f-4ec0-b869-e0b5d9c5f433-23.jpg?height=399&width=523&top_left_y=356&top_left_x=769)
Fig. 5 A real inverted pendulum.

The human stick balancing game motivates us to reconsider certain issues of stabilizing the inverted pendulum. Conventionally, the inverted pendulum is equipped with two sensors, i.e., the cart position sensor and the rod angle sensor. The feedback stabilization of the inverted pendulum is usually done by using the measured two sensor outputs. If we recall the stick balancing game, it is highly unlikely that our eyes are focused on the finger position and the stick angle simultaneously. What do we really look at when we try to balance a stick using our hand? The researchers now tend to believe that the player in the game looks at the top end of the stick when the player tries to move the fingers [Qiu and Zhou, 2009], [Leong and Doyle, 2016], [Doyle et al., 2016]. To mimic this human behavior, we utilize a single position sensor, which measures the horizontal position of the upper tip of the rod, to achieve the stabilization of the inverted pendulum. Clearly, such an inverted pendulum system would be not only under-actuated, but also under-sensed. The control of such a system is much more challenging compared with controlling an inverted pendulum by using two measured outputs.

In this section, the output feedback stabilization of an under-sensed and underactuated linear (USUAL) inverted pendulum, which has only one position sensor and one force actuator, is investigated. We successfully stabilize this USUAL inverted pendulum without sophisticated tuning. The optimally robust controller introduced earlier is demonstrated to be effective. To the best of our knowledge, this is the first successful experimental study on controlling a linear inverted pendulum by using a single position sensor measurement.

### 5.1 System Model

As shown in Fig. 6, a standard linear inverted pendulum consists of a cart and a rod. The cart, with a mass $M_{c}$, slides on a stainless shaft and is equipped with a linear motor. The rod, attached with a small ball, is mounted on the cart. The axis of rotation of the rod is perpendicular to the direction of the motion of the cart. The
rod, of length $L$, has an evenly distributed mass $M_{p}$, and the small ball with a mass $M_{b}$ can be regarded as a point mass. The system has two degrees of freedom. One is from the horizontal motion of the cart, and the other is from the rotational motion of the rod on the plane. Nevertheless, only the horizontal motion of the cart is actuated by the force $f(t)$ applied to the cart, and only the horizontal position of the tip of the $\operatorname{rod} z(t)$ is measured by a single position sensor. Consequently, Fig. 6 indeed shows the schematic diagram of the USUAL inverted pendulum.

![](https://cdn.mathpix.com/cropped/e749505a-107f-4ec0-b869-e0b5d9c5f433-24.jpg?height=339&width=412&top_left_y=721&top_left_x=820)
Fig. 6 A schematic diagram of the USUAL inverted pendulum with input $f(t)$ and output $z(t)$.

The differential equation model [Qiu and Zhou, 2009, Section 2.10 and Section 3.9] of the USUAL inverted pendulum is given by

$$
\begin{align*}
f(t) & =M_{1} \ddot{x}(t)-M_{2} L \ddot{\theta}(t) \cos \theta(t)+M_{2} L \dot{\theta}^{2}(t) \sin \theta(t) \\
0 & =M_{3} L \ddot{\theta}(t)-M_{2} \ddot{x}(t) \cos \theta(t)-M_{2} g \sin \theta(t)  \tag{25}\\
z(t) & =x(t)-L \sin \theta(t)
\end{align*}
$$

where $f(t)$ is the system input, $z(t)$ is the system output, $x(t)$ is the cart position, $\theta(t)$ is the pendulum angle, $g$ is the gravitational acceleration, $M_{1}=M_{p}+M_{c}+ M_{b}, M_{2}=M_{p} / 2+M_{b}$, and $M_{3}=M_{p} / 3+M_{b}$ are three constant coefficients of the practical system. The system given in (25) is highly nonlinear. Our control objective is to stabilize the rod around its upward direction, which is an unstable equilibirium point. Linearizing the system around the equilibrium point $x(t)=0, \dot{x}(t)=0, \theta(t)= 0, \dot{\theta}(t)=0$ yields

$$
\begin{aligned}
f(t) & =M_{1} \ddot{x}(t)-M_{2} L \ddot{\theta}(t) \\
0 & =M_{3} L \ddot{\theta}(t)-M_{2} \ddot{x}(t)-M_{2} g \theta(t), \\
z(t) & =x(t)-L \theta(t)
\end{aligned}
$$

together with the transfer function $P(s)$ from $F(s)$ to $Z(s)$ as

$$
\begin{equation*}
P(s)=\frac{\left(M_{3} / M_{2}-1\right) L s^{2}-g}{M_{1} s^{2}\left[\left(M_{3} / M_{2}-M_{2} / M_{1}\right) L s^{2}-g\right]} \tag{26}
\end{equation*}
$$

Plugging the actual values of the parameters given in Table 1 into (26) results in

Table 1 Parameters of the USUAL inverted pendulum in our experimental set-up.
| Parameter | Value |
| :--- | :--- |
| Mass of rod $\left(M_{p}\right)$ | 0.07 kg |
| Mass of the cart $\left(M_{c}\right)$ | 1.42 kg |
| Mass of the ball $\left(M_{b}\right)$ | 0.05 kg |
| Gravitational acceleration $(g)$ | $9.8 \mathrm{~m} / \mathrm{s}^{2}$ |
| Length of the rod $(L)$ | 0.335 m |


$$
\begin{equation*}
P(s)=\frac{-0.1104 s^{2}-22.52}{s^{2}\left(s^{2}-36.23\right)} . \tag{27}
\end{equation*}
$$

This is a highly unstable system with poles at $0,0, \pm 6.019$ and zeros at $\pm j 14$.28. It is impossible to stabilize this system by using PD or PID control.

### 5.2 Optimally Robust Stabilization

In real applications, before we apply the design algorithm to $P(s)$, generally, loopshaping for the plant $P(s)$ is carried out to help improve the control performance. The purpose of shaping the plant is to balance the system input and output possibly using a frequency dependent weighting function. In our experimental set-up, a simple weighting constant is demonstrated to be sufficient.

Specifically, for the USUAL inverted pendulum, first multiply the original plant $P(s)$ given in (27) by the simplest weighting function, i.e., a constant $W$, to form a new plant $\hat{P}(s)=W P(s)$, then carry out the optimally robust stabilization algorithm to obtain the resulting controller $\hat{C}(s)$. Note that the loop transfer function is given by

$$
L(s)=\hat{P}(s) \hat{C}(s)=P(s) W \hat{C}(s)=P(s) C(s)
$$

where $C(s)=W \hat{C}(s)$. In other words, to guarantee the same loop transfer function for the original $P(s)$, we need to absorb the weighting constant $W$ into the controller $C(s)$. As a result, $C(s)$ is the optimally robust controller that we use in reality for the original plant $P(s)$.

The shaping constant $W$ should be carefully tuned in actual applications. In our real USUAL inverted pendulum set-up, we find that a large range of $W$ is applicable, and we set $W=400$ for our experiment.

In the following, we make use of the main algorithm to design an optimally robust stabilizing controller for the real USUAL inverted pendulum.

Example 3 (USUAL Inverted Pendulum). Consider the shaped plant

$$
\hat{P}(s)=W P(s)=\frac{400\left(-0.1104 s^{2}-23.52\right)}{s^{4}-36.23 s^{2}} .
$$

Following Algorithm 1, we try to find the optimal $\hat{C}(s)$ such that $\|\hat{P}(s) \# \hat{C}(s)\|_{\infty}$ is minimized.

1. (Spectral factorization)

$$
\left(s^{4}-36.23 s^{2}\right)^{2}+400^{2}\left(-0.1104 s^{2}-23.52\right)^{2}=d(-s) d(s)
$$

This yields $d(s)=s^{4}+27.21 s^{3}+334.0 s^{2}+2335 s+9409$.
2. (Matrix computation) We can compute that

$$
\boldsymbol{H}=\left[\begin{array}{cccc}
-2.073 & -0.3017 & -2.962 \times 10^{-2} & -1.476 \times 10^{-3} \\
-45.48 & -6.136 & -0.5042 & -1.055 \times 10^{-2} \\
-425.9 & -55.28 & -3.755 & 1.115 \times 10^{-2} \\
-2839 & -278.7 & -13.89 & 0.3075
\end{array}\right]
$$

3. (Eigen-computation) The four eigenvalues of $\boldsymbol{H}$ are $-13.20,1.977,-0.5058$ and $7.574 \times 10^{-2}$. The one with the largest magnitude is $\rho(\boldsymbol{H})=13.20$ and its corresponding eigenvector is

$$
\boldsymbol{e}=\left[-1.315 \times 10^{-3}-2.416 \times 10^{-2}-0.1994-0.9796\right]^{T}
$$

This gives

$$
e(s)=\left[\begin{array}{llll}
s^{3} & s^{2} & s & 1
\end{array}\right] \boldsymbol{e}=-1.315 \times 10^{-3} s^{3}-2.416 \times 10^{-2} s^{2}-0.1994 s-0.9796
$$

4. (Pole placement) The pole placement equation follows that

$$
\left(s^{4}-36.23 s^{2}\right) p(s)+400\left(-0.1104 s^{2}-23.52\right) q(s)=d(s) e(s)
$$

Solving the above equation gives

$$
\begin{aligned}
& p(s)=s^{3}+45.57 s^{2}+438.6 s+9834 \\
& q(s)=-13.20 s^{3}-116.8 s^{2}-336.5 s-744.8 .
\end{aligned}
$$

Therefore, the pole placement controller is given by

$$
\hat{C}(s)=\frac{q(s)}{p(s)}=\frac{-13.20 s^{3}-116.8 s^{2}-336.5 s-744.8}{s^{3}+45.57 s^{2}+438.6 s+9834}
$$

Absorbing $W$ yields the desired optimally robust controller in practical use

$$
\begin{equation*}
C(s)=W \hat{C}(s)=\frac{400\left(-13.20 s^{3}-116.8 s^{2}-336.5 s-744.8\right)}{s^{3}+45.57 s^{2}+438.6 s+9834} \tag{28}
\end{equation*}
$$

5. (Optimal robustness margin computation) We have

$$
\alpha(\hat{P}(s))=7.552 \times 10^{-2}
$$

### 5.3 Simulation and Experimental Results

To verify the effectiveness of the design algorithm, we first show the simulation results for the nonlinear model given in (25) together with the optimally robust controller $C(s)$ designed in (28). Given the initial conditions $x(t)=0.01 \mathrm{~m}, \dot{x}(t)= 0.002 \mathrm{~m} / \mathrm{s}, \theta(t)=4 \pi / 180 \mathrm{rad}, \dot{\theta}(t)=0.5 \pi / 180 \mathrm{rad} / \mathrm{s}$, the stabilization simulation results are shown in Fig. 7.

The closed-loop system starts with the initial conditions, and the simulation results show that the output $z(t)$, the cart position $x(t)$ and pendulum angle $\theta(t)$ converge to zero quickly when $t>2 \mathrm{~s}$. This validates the effectiveness of the designed controller from a theoretical perspective.

![](https://cdn.mathpix.com/cropped/e749505a-107f-4ec0-b869-e0b5d9c5f433-27.jpg?height=970&width=803&top_left_y=934&top_left_x=640)
Fig. 7 The nonlinear simulation of the USUAL inverted pendulum: stabilization results of the output $z(t)$, cart position $x(t)$ and pendulum angle $\theta(t)$ with the initial conditions $x(t)=0.01 \mathrm{~m}$, $\dot{x}(t)=0.002 \mathrm{~m} / \mathrm{s}, \theta(t)=4 \pi / 180 \mathrm{rad}, \dot{\theta}(t)=0.5 \pi / 180 \mathrm{rad} / \mathrm{s}$.

In the following, we implement the optimally robust stabilizing controller $C(s)$ given in (28) to the real USUAL inverted pendulum. The experiment is carried out
as follows. In the beginning, we show the stabilized behaviors of $z(t), x(t)$ and $\theta(t)$ of the USUAL inverted pendulum. Then, we excite the system by knocking the pendulum gently on the top as performance testing. In the end, the behaviors of $z(t)$, $x(t)$ and $\theta(t)$ of the system against the knock are presented.

The real-time experimental data of three variables $z(t), x(t)$ and $\theta(t)$ together with the performance testing are illustrated by Fig. 8. When $t<9.7 \mathrm{~s}$, the output $z(t)$ is within a small range $[-0.04 \mathrm{~m}, 0.02 \mathrm{~m}]$, and from $x(t)$ and $\theta(t)$, we know that the real USUAL inverted pendulum is indeed stabilized. Moreover, both $\theta(t)$ and $x(t)$ vary within a small range. The results indicate the closed-loop system is running with satisfactory stabilized behaviors.

![](https://cdn.mathpix.com/cropped/e749505a-107f-4ec0-b869-e0b5d9c5f433-28.jpg?height=970&width=806&top_left_y=872&top_left_x=640)
Fig. 8 The stabilization of the real USUAL inverted pendulum: results of the output $z(t)$, cart position $x(t)$ and pendulum angle $\theta(t)$. The circles represent the rough time $t=9.7 \mathrm{~s}$ when we excite the system by hitting the pendulum on the top.

In order to test the system performance, we excite the system by hitting the pendulum lightly on the top when the designed controller is in operation. As shown in Fig. 8, the circles represent the rough time $t=9.7 \mathrm{~s}$ when we excite the system. The results show that both $x(t)$ and $\theta(t)$ restore quickly to their stabilized behaviors.

In the meantime, $z(t)$ is almost free of the impact of the hit since $z(t)$ is the controlled output. This validates the effectiveness of the designed controller from a practical point of view.

By simply shaping the USUAL inverted pendulum with a constant to balance the system input and output, the optimally robust controller can be implemented successfully without further complicated tuning. We conclude that the optimally robust control is demonstrated to be effective in the control of the USUAL inverted pendulum.

## 6 Conclusion

To characterize system uncertainties of different types and from multiple sources, we have proposed a special uncertainty model, namely, the uncertainty quartet. The uncertainty quartet combines and generalizes several commonly adopted uncertainty models, such as the additive, the multiplicative, the relative and the feedback uncertainties. In correspondence with the uncertainty quartet, a robust stability condition was derived, resulting in a robust stability margin in terms of the Gang of Four transfer matrix. An optimally robust controller, maximizing the robust stability margin, was obtained through a proposed polynomial approach. This approach involves only basic matricial and polynomial manipulations. Moreover, the mathematical tools used in developing this polynomial approach are also rudimentary, e.g., the matrix analysis and basic $\mathscr{H}_{\infty}$ control theory. The clarity and simplicity of the polynomial approach may be beneficial to the popularization of the robust control theory for engineering applications.

The optimally robust controller was demonstrated to be effective by the case study on the USUAL inverted pendulum, a highly nonlinear and unstable singleinput single-output system. This system is commonly seen in laboratories and familiar to most of people in the field of control. It is nontrivial to control such a system with simple methods, such as, PID control. As a result, the USUAL inverted pendulum may be regarded as a benchmark to validate the effectiveness of control methods in practice. For the purpose of education, the control of this system may serve as a qualifying test for control system designers and engineers.

## Acknowledgement

Useful discussions with Dr. Wei Chen on the writing of this chapter are gratefully acknowledged.

This work was supported in part by the Research Grants Council of Hong Kong Special Administrative Region, China, under Project 16201115 and ThemeBased Research Scheme T23-701/14-N, and in part by the government of Jiangmen, Guangdong, China.

## References

Åström, K. J. and Murray, R. M. (2008). Feedback Systems: An Introduction for Scientists and Engineers. Princeton, NJ: Princeton University Press.
Doyle, J. C., Francis, B. A., and Tannenbaum, A. R. (1990). Feedback Control Theory. London: Macmillan Publishers Ltd.
Doyle, J. C., Glover, K., Khargonekar, P. P., and Francis, B. A. (1989). State-space solutions to standard $\mathscr{H}_{2}$ and $\mathscr{H}_{\infty}$ control problems. IEEE Trans. Automat. Contr., 34(8):831-847.
Doyle, J. C., Nakahira, Y., Leong, Y. P., Jenson, E., Dai, A., Ho, D., and Matni, N. (2016). Teaching control theory in high school. In Proc. 55th IEEE Conf. on Decision and Contr. (CDC), pages 5925-5949.
Fuhrmann, P. A. (2012). A Polynomial Approach to Linear Algebra. New York, NY: Springer Science \& Business Media.
Georgiou, T. T. and Smith, M. C. (1990). Optimal robustness in the gap metric. IEEE Trans. Automat. Contr., 35(6):673-686.
Georgiou, T. T. and Smith, M. C. (1997). Robustness analysis of nonlinear feedback systems: an input-output approach. IEEE Trans. Automat. Contr.,, 42(9):1200-1221.
Gu, G. and Qiu, L. (1998). Connection of multiplicative/relative perturbation in coprime factors and gap metric uncertainty. Automatica, 34(5):603-607.
Gu, G. and Qiu, L. (2011). A two-port approach to networked feedback stabilization. In Proc. 50th IEEE Conf. on Decision and Contr. and European Contr. Conf. (CDC-ECC), pages 2387-2392.
Halsey, K. M. and Glover, K. (2005). Analysis and synthesis of nested feedback systems. IEEE Trans. Automat. Contr., 50(7):984-996.
Kailath, T. (1980). Linear Systems. NJ: Prentice-Hall Englewood Cliffs.
Kanno, M. (2003). Guaranteed Accuracy Computations in Systems and Control. PhD thesis, University of Cambridge.
Lanzon, A. and Papageorgiou, G. (2009). Distance measures for uncertain linear systems: A general theory. IEEE Trans. Automat. Contr., 54(7):1532-1547.
Leong, Y. P. and Doyle, J. C. (2016). Understanding robust control theory via stick balancing. In Proc. 55th IEEE Conf. on Decision and Contr. (CDC), pages 1508-1514.
Liang, Y. and Qiu, L. (2009). A polynomial solution to an $\mathscr{H}_{\infty}$ robust stabilization problem. In Proc. 7th IEEE Asian Contr. Conf. (ASCC), pages 642-647.
Liu, K.-Z. and Yao, Y. (2016). Robust Control: Theory and Applications. Singapore: John Wiley \& Sons.
McFarlane, D. and Glover, K. (1990). Robust Controller Design Using Normalized Coprime Factor Plant Descriptions. New York: Springer-Verlag.
Nehari, Z. (1957). On bounded bilinear forms. Annals of Mathematics, pages 153162.

Petersen, I. R. and Tempo, R. (2014). Robust control of uncertain systems: Classical results and recent developments. Automatica, 50(5):1315-1335.

Qiu, L. and Davison, E. J. (1992a). Feedback stability under simultaneous gap metric uncertainties in plant and controller. Syst. Contr. Lett., 18(1):9-22.
Qiu, L. and Davison, E. J. (1992b). Pointwise gap metrics on transfer matrices. IEEE Trans. Automat. Contr., 37(6):741-758.
Qiu, L. and Zhou, K. (2009). Introduction to Feedback Control. Upper Saddle River, NJ: Prentice Hall.
Vidyasagar, M. (1985). Control System Synthesis: A Factorization Approach. Cambridge, MA: M.I.T. Press.
Vinnicombe, G. (1993). Frequency domain uncertainty and the graph topology. IEEE Trans. Automat. Contr., 38(9):1371-1383.
Vinnicombe, G. (2000). Uncertainty and Feedback: $\mathscr{H}_{\infty}$ Loop-shaping and the vgap Metric. Singapore: World Scientific.
Youla, D. C., Bongiorno, J. J., and Jabr, H. A. (1976). Modern Wiener-Hopf design of optimal controllers: part I. IEEE Trans. Automat. Contr., 21(1):3-13.
Zames, G. and El-sakkary, A. K. (1980). Unstable systems and feedback: the gap metric. In Proc. 16th Allerton Conf., pages 380-385.
Zhao, D. and Qiu, L. (2016). Networked robust stabilization with simultaneous uncertainties in plant, controller and communication channels. In Proc. 55th IEEE Conf. on Decision and Contr. (CDC), pages 2376-2381.
Zhou, K. and Doyle, J. C. (1998). Essentials of Robust Control. Upper Saddle River, NJ: Prentice Hall.


[^0]:    Di Zhao, Chao Chen and Li Qiu are with the Department of Electronic and Computer Engineering, The Hong Kong University of Science and Technology, Clear Water Bay, Kowloon, Hong Kong, China (dzhaoaa@ust.hk, cchenap@ust.hk, eeqiu@ust.hk).
    Sei Zhen Khong is with the Department of Electrical and Electronic Engineering, The University of Hong Kong, Pokfulam, Hong Kong, China. The work was mainly completed when he was a visitor to The Hong Kong University of Science and Technology (szkhong@hku.hk).

[^1]:    ${ }^{1}$ It can be shown that all the eigenvalues of $\boldsymbol{H}$ are real. For clarity of presentation, it is implicitly assumed that there exists a unique eigenvalue of $\boldsymbol{H}$ whose magnitude is $\rho(\boldsymbol{H})$. This is generically the case. The more involved situation is discussed specifically in Section 3.3.

