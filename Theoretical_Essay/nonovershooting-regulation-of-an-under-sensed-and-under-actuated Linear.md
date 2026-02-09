# Nonovershooting Regulation of an Under-Sensed and Under-Actuated Linear Inverted Pendulum 

Chao Chen, Di Zhao and Li Qiu


#### Abstract

This paper studies a nonovershooting regulation problem of an under-sensed and under-actuated system using two-degree-of-freedom (2DOF) control. An under-sensed and under-actuated linear inverted pendulum, which only has a single position sensor unlike the standard set-up, is investigated. For this system, first we present a regulation task that comprises three requirements, namely a nonovershooting step response, the largest robust stability margin and a 2DOF controller of the minimum order. Then we study the issue of controller design and develop a fourth-order 2DOF controller via the Youlaparameterization and the loop-shaping approaches. Finally, the proposed 2DOF controller is demonstrated to be effective.


## I. INTRODUCTION

Under-actuation, meaning that the system owns a lower number of actuators than the degrees of freedom, is a common property existing in mechanical control systems. To control an under-actuated mechanical system, we often adopt full state feedback or full position feedback. This requires that the number of sensors the control system needs is at least equal to the number of position variables, namely the degrees of freedom. If a lower number of sensors is used, the system would be under-sensed. Under-sensed systems and under-actuated systems can be regarded as a dual pair even though the attention on controlling under-sensed systems is rarely seen in the literature. To attract greater attention on this dual pair, in this paper we study the control of a real system which is naturally under-sensed and under-actuated.

Over the past few decades, the inverted pendulum, as a typical under-actuated system, has been one of the most popular control education apparatus. Control of the inverted pendulum simulates a childhood game where a long stick is balanced upward by our fingertips. Conventionally, the inverted pendulum is equipped with two sensors, namely a cart position sensor and a rod angular position sensor, and the feedback stabilization is usually done by utilizing two sensor outputs. If we recall the stick balancing game, it is highly unlikely that our eyes are focused on the fingertip position and the stick angle simultaneously. What do we really look at for the purpose of balancing the stick? The researchers in [1], [2], [3] now tend to believe that we only look at a certain point on the stick, for instance, the top end of the stick, when we move our fingertips. This motivates us to control

[^0]a real inverted pendulum using a single position sensor. An Under-Sensed and Under-Actuated Linear (USUAL) inverted pendulum, which owns a designable single position sensor, is introduced in our previous work [4]. To control such a system, we need to know where to place the configurable sensor for the ease of control and how to design the controller effectively under incomplete position information. The authors demonstrate that the configurable sensor location has a significant impact on the robust stability of the controlled USUAL inverted pendulum.

Inheriting from the above work, in this paper, we study a nonovershooting regulation synthesis problem of the USUAL inverted pendulum. The issue of designing nonvershooting control systems is of great importance in many applications. For example, a manipulator performs pick and place operations close to a wall [5]. This seemingly easy issue, in fact, is difficult and has not been completely solved yet. To the best of our knowledge, even for a linear timeinvariant (LTI) system, in the literature there does not exist a necessary and sufficient characterization of the transfer function having a nonovershooting step response. Several remarkable attempts include [6], [7], [8] for LTI system analysis, [5], [9], [10] for LTI system synthesis, and [11] for nonlinear system synthesis. One effective way to meet the nonvershooting requirement in a regulation problem is to utilize two-degree-of-freedom (2DOF) control [1], [12]. In particular, for a discrete-time single-input-single-output (SISO) LTI system, it is pointed out in [5] that there exist 2DOF controllers ensuring a nonovershooting step response. The authors also mention that analogous formulations can be derived for the continuous-time counterpart. However, the resulting controllers would always be irrational which are difficult to implement in practice. To overcome this shortcoming, the authors in [9] propose an approach to constructing a rational 2DOF controller for the continuoustime counterpart. It is noteworthy that the order constraint of resulting controllers is not taken into account in [1], [9], [12]. In fact, a high-order 2DOF controller is subsequently inevitable in their examples. The order of controllers is directly related to controller complexity and is a critical specification in the control system design. It is often desirable to have low-order controllers in many applications if the performance requirement can be guaranteed.

In this paper, we investigate a synthesis problem of the 2DOF regulation of the USUAL inverted pendulum. The problem contains three requirements which are a nonovershooting step response, the largest robust stability margin and the minimum order constraint of a 2DOF controller. For
general systems, it is certainly hard to solve a minimum order controller problem with nonovershooting and robust stability constraints. However, it is solvable for the USUAL inverted pendulum to derive a fourth-order 2DOF controller.

The outline of this paper is as follows. In Section II, the model of the USUAL inverted pendulum and the problem formulation are presented. Section III provides the main results which give a set of optimal solutions to the synthesis problem using the Youla-parameterization method. Moreover, the proposed 2DOF controller is demonstrated to be effective. Section IV concludes this paper.

## II. PROBLEM FORMULATION

## A. System modeling

A standard inverted pendulum, illustrated by Fig. 1, consists of a cart and a rod. A cart slides on a stainless shaft and is equipped with a linear motor. A rod with a length $L$, attached with a small ball, is mounted on the cart. The axis of rotation of the rod is perpendicular to the direction of the motion of the cart. The inverted pendulum has two degrees of freedom but it is under-actuated due to the single input $f(t)$ applied to the cart.

Motivated by the scenario of the stick balancing game, we wish to accomplish the control of the inverted pendulum using a single position sensor. As is shown in Fig. 1, we denote the single configurable sensor location as $\alpha L$ where $\alpha \in[0, \infty)$ is a tunable variable. The range of $\alpha$ implies the sensor location can be shifted from the bottom of the rod towards the ray of the rod. We take the horizontal position $z_{\alpha}(t)$ of the corresponding sensor location $\alpha L$ as the single output. Such a system, named the USUAL inverted pendulum if the linear model is considered, would be not only underactuated but also under-sensed.

![](https://cdn.mathpix.com/cropped/cfa53b96-c79f-4c5c-b628-d83ae7cd91ce-2.jpg?height=339&width=392&top_left_y=1645&top_left_x=397)
Fig. 1. An inverted pendulum

TABLE I
List of parameters
| Parameter | Value |
| :---: | :---: |
| Mass of rod $\left(M_{p}\right)$ | 0.07 kg |
| Mass of the cart $\left(M_{c}\right)$ | 1.42 kg |
| Mass of the ball $\left(M_{b}\right)$ | 0.05 kg |
| Gravitational acceleration $(g)$ | $9.8 \mathrm{~m} / \mathrm{s}^{2}$ |
| Length of the rod $(L)$ | 0.335 m |


The nonlinear differential equation model [1, Section 2.10] of the USUAL inverted pendulum is given by

$$
\begin{align*}
f(t) & =M_{1} \ddot{x}(t)-M_{2} L \ddot{\theta}(t) \cos \theta(t)+M_{2} L \dot{\theta}^{2}(t) \sin \theta(t) \\
0 & =M_{3} L \ddot{\theta}(t)-M_{2} \ddot{x}(t) \cos \theta(t)-M_{2} g \sin \theta(t)  \tag{1}\\
z_{\alpha}(t) & =x(t)-\alpha L \sin \theta(t)
\end{align*}
$$

where $f(t)$ is the force input, $z_{\alpha}(t)$ is the position output, $x(t)$ is the cart position, $\theta(t)$ is the pendulum angle, and $g$ is the gravitational acceleration. $M_{1}=M_{p}+M_{c}+M_{b}$, $M_{2}=M_{p} / 2+M_{b}$ and $M_{3}=M_{p} / 3+M_{b}$ are three constant coefficients of the system where $M_{c}, M_{p}, M_{b}$ are the mass of the cart, the rod, the ball, respectively. The stabilization of the inverted pendulum is to keep the rod upward where the linearization process can be taken. Therefore, we linearize (1) around the equilibrium point $x(t)=0, \dot{x}(t)=0, \theta(t)=0$, $\dot{\theta}(t)=0$ and obtain the transfer function from $f(t)$ to $z_{\alpha}(t)$ :
$P_{\alpha}(s)=\frac{\left(M_{3} / M_{2}-\alpha\right) L s^{2}-g}{M_{1} s^{2}\left[\left(M_{3} / M_{2}-M_{2} / M_{1}\right) L s^{2}-g\right]}, \alpha \in[0, \infty)$.
In our experimental set-up, the values of the parameters are given in Table I. The real system model is given by

$$
\begin{equation*}
P_{\alpha}(s)=\frac{0.694\left[(1-1.159 \alpha) s^{2}-33.908\right]}{s^{2}\left(s^{2}-36.225\right)}, \alpha \in[0, \infty) . \tag{2}
\end{equation*}
$$

Since $P_{\alpha}(s)$ contains double integrators, if we use the well-known unity feedback control, the type of the loop transfer function is at least two provided that unstable polezero cancellation is not allowed. According to [1, Theorem 4.20], the step response of $P_{\alpha}(s)$ will invariably exhibit an overshoot in a unity feedback system. This drives us to adopt a 2DOF structure as a default in the nonovershooting regulation problem as detailed below.

## B. Two-degree-of-freedom controller

The regulation problem, shown in Fig. 2, can be formulated into the following sense: Given plant $P(s)$, design controller $\boldsymbol{C}(s)=\left[C_{1}(s) \quad C_{2}(s)\right]$ so that good performance in tracking and disturbance rejection is achieved. Here $\boldsymbol{C}(s)$ is a double-input-single-output system containing a feedforward controller $C_{1}(s)$ and a feedback controller $C_{2}(s)$. Such a system is named a 2DOF controller. One of the advantages of the 2DOF control mechanism, as stated in [1, Theorem 4.23], is that nonovershooting tracking can be guaranteed with an appropriate 2DOF controller. Such a 2DOF controller is constructed in [1], [9]. We adopt the main idea from this construction method and modify it to meet the demands in our paper. We use the Youla-parametrization of all stabilizing 2DOF controllers introduced in [13], [14].

Let the plant

$$
P(s)=\frac{b(s)}{a(s)}=\frac{b_{0} s^{n}+b_{1} s^{n-1}+\cdots+b_{n}}{a_{0} s^{n}+a_{1} s^{n-1}+\cdots+a_{n}}
$$

where $a(s)$ and $b(s)$ are coprime and $a_{0} \neq 0$. Let

$$
C_{0}(s)=\frac{q(s)}{p(s)}=\frac{q_{0} s^{m}+q_{1} s^{m-1}+\cdots+q_{m}}{p_{0} s^{m}+p_{1} s^{m-1}+\cdots+p_{m}}
$$

![](https://cdn.mathpix.com/cropped/cfa53b96-c79f-4c5c-b628-d83ae7cd91ce-3.jpg?height=334&width=828&top_left_y=176&top_left_x=197)
Fig. 2. A feedback system for regulation

where $p(s)$ and $q(s)$ are coprime and $p_{0} \neq 0$, be any initial stabilizing feedback controller. Factorize the characteristic polynomial $c(s):=a(s) p(s)+b(s) q(s)$ as $c(s)=f(s) h(s)$ such that $\operatorname{deg} f(s)=n$ and $\operatorname{deg} h(s)=m$. Let
$M(s)=\frac{a(s)}{f(s)}, N(s)=\frac{b(s)}{f(s)}, X(s)=\frac{p(s)}{h(s)}, Y(s)=\frac{q(s)}{h(s)}$,
Note that $M(s) X(s)+N(s) Y(s)=1$. We denote the set of all stabilizing 2DOF controllers of $P(s)$ by $\mathcal{T}(P)$ given by

$$
\begin{align*}
& \mathcal{T}(P):= \\
& \left\{\boldsymbol{C}(s)=\left[\frac{Q_{1}(s)}{X(s)-N(s) Q_{2}(s)} \frac{Y(s)+M(s) Q_{2}(s)}{X(s)-N(s) Q_{2}(s)}\right]:\right. \\
& \left.Q_{1}(s) \text { and } Q_{2}(s) \text { are stable }\right\} . \tag{3}
\end{align*}
$$

## C. Robust stability

For the feedback system in Fig. 2, the Gang of Four matrix formed by $P(s)$ and $C_{2}(s)$, introduced in [15], is defined as

$$
P(s) \# C_{2}(s):=\left[\begin{array}{cc}
\frac{1}{1+P(s) C_{2}(s)} & \frac{C_{2}(s)}{1+P(s) C_{2}(s)} \\
\frac{P(s)}{1+P(s) C_{2}(s)} & \frac{P(s) C_{2}(s)}{1+P(s) C_{2}(s)}
\end{array}\right] .
$$

It is noteworthy that $P(s) \# C_{2}(s)$ is essential for studying the robust stability issue of the feedback loop in Fig. 2. To be specific, it is well accepted that $\left\|P(s) \# C_{2}(s)\right\|_{\infty}^{-1}$ can be regarded as a robust stability margin of the feedback system. The optimally robust $C_{2}(s)$ thereupon is attained by minimizing $\left\|P(s) \# C_{2}(s)\right\|_{\infty}$. We refer the readers to [16] for more details of the robust stability issue. For a SISO plant $P(s)$, it has been proved in [17, Section 6.2] that the corresponding optimally robust $C_{2}(s)$ is unique.

The loop-shaping method [18] is often adopted for the purpose of improving the control performance. This method provides extra design freedom to tune $P(s)$ if the unique optimally robust $C_{2}(s)$ does not work well in real applications. The main idea of this method is as follows. For a given $P(s)$ and a given stable proper $W(s)$, let $\hat{P}(s)=W(s) P(s)$. Here $P(s)$ is the real plant and $\hat{P}(s)$ is the notational, weighted plant that will be utilized for the design purpose. Denote $C_{2}(s)=\hat{C}_{2}(s) W(s)$ where the optimally robust $\hat{C}_{2}(s)$ is obtained by minimizing $\left\|\hat{P}(s) \# \hat{C}_{2}(s)\right\|_{\infty}$. It should be
noted that $C_{2}(s)$ is the real controller that we use for $P(s)$ while $\hat{C}_{2}(s)$ is the notional, interim controller. In general, the shaping weight $W(s)$ should be chosen such that the weighted plant $\hat{P}(s)$ satisfies the certain requirement on the specification of the system frequency response. For the USUAL inverted pendulum $P_{\alpha}(s)$, a simple constant weight $W(s)=W$ is adopted in order to balance the system input and output. This constant weight is demonstrated to be sufficient in practice.

## D. Problem formulation

Based on the parameterization in (3), we derive the transfer function from the reference input $r(t)$ to the output $z(t)$, which is given by

$$
\begin{align*}
G(s) & =\frac{P(s) C_{1}(s)}{1+P(s) C_{2}(s)}=\frac{N(s) Q_{1}(s)}{M(s) X(s)+N(s) Y(s)} \\
& =N(s) Q_{1}(s) \tag{4}
\end{align*}
$$

Notice that $G(s)$ depends only on $Q_{1}(s)$ and reflects the tracking performance. This shows that $Q_{2}(s)$ has nothing to do with $G(s)$ and hence can be designed independently. Furthermore, the disturbance response from $d(t)$ to $z(t)$ and the noise response from $n(t)$ to $z(t)$, both reflecting the robust stability, depend only on $Q_{2}(s)$. This makes choosing $Q_{1}(s)$ and choosing $Q_{2}(s)$ decoupled and rather convenient. Now we are ready for the problem formulation.

We consider a 2DOF regulation problem of the USUAL inverted pendulum $P_{\alpha}(s)$ which satisfies the following requirements.

1) The output response to the step reference does not have an overshoot.
2) The closed-loop system is optimally robust stable.
3) The minimum order of 2DOF controller is achieved. Given a shaping constant $W$, we denote $\hat{P}_{\alpha}(s)=W P_{\alpha}(s)$ and $\hat{C}_{2}(s)=C_{2}(s) W^{-1}$. Specifically, the above problem can be restated by the following mathematical problem:

For $P_{\alpha}(s)$ in (2) and a given $W$, find a minimum order 2DOF controller $\boldsymbol{C}_{\alpha}(s)=\left[C_{1}(s) C_{2}(s)\right]$ such that the output $z_{\alpha}(t) \leq 1$ for all $t \geq 0$ when $r(t)$ is a unit step input $\sigma(t)$ and $\left\|\hat{P}_{\alpha}(s) \# \hat{C}_{2}(s)\right\|_{\infty}$ is minimized simultaneously.

## III. MAIN RESULTS

As mentioned in the problem formulation, the design of $C_{1}(s)$ and $C_{2}(s)$ for $P_{\alpha}(s)$ is decoupled since $Q_{1}(s)$ and $Q_{2}(s)$ can be designed independently. The main design procedure is as follows. First, design $Q_{2}(s)$ to obtain an optimally robust stabilizing $C_{2}(s)$ which incorporates a constant weight $W$ by minimizing $\mathcal{H}_{\infty}$ norm of $\hat{P}_{\alpha}(s) \# \hat{C}_{2}(s)$. Then, for this fixed $C_{2}(s)$, design $Q_{1}(s)$ to make the step response of $G(s)$ nonovershooting and to attain a minimum order $\boldsymbol{C}_{\alpha}(s)$ based on the $C_{2}(s)$ at the same time.

The former is a standard $\mathcal{H}_{\infty}$ control problem which has been demonstrated for $P_{\alpha}(s)$ when $\alpha \in[0.9,2.5]$ in our previous work [4]. For the sake of simplicity, we consider the case when the configurable position sensor located at $\alpha=1.5$ hereinafter. For other choices of $\alpha \in[0.9,2.5]$, $\boldsymbol{C}_{\alpha}(s)$ can be derived in an analogous way.

## A. Controller design procedure

We first design $C_{2}(s)$. Substituting $\alpha=1.5$ into (2) yields

$$
P_{\alpha}(s)=\frac{-0.5125\left(s^{2}+45.915\right)}{s^{2}\left(s^{2}-36.225\right)}=: \frac{b(s)}{a(s)} .
$$

Shape $P_{\alpha}(s)$ by the constant $W=300$, and then solve the optimally robust stabilizing problem of the weighted plant $W P_{\alpha}(s)$ to attain $\hat{C}_{2}(s)$. It follows that a third-order controller $C_{2}(s)=\hat{C}_{2}(s) W=: \frac{q(s)}{p(s)}$ is given by

$$
\begin{equation*}
C_{2}(s)=\frac{-1520 s^{3}-1.439 e 04 s^{2}-4.623 e 04 s-1.237 e 05}{s^{3}+40.65 s^{2}+2.664 s+2089} \tag{5}
\end{equation*}
$$

This $C_{2}(s)$ is the unique optimally robust stabilizing controller for $P_{\alpha}(s)$ up to the shaping weight $W$. Factorize $c(s)=a(s) p(s)+b(s) q(s)$ such that

$$
\begin{align*}
& c(s)=f(s) h(s) \\
& f(s)=s^{4}+25.06 s^{3}+277.7 s^{2}+1323 s+7057  \tag{6}\\
& h(s)=s^{3}+15.59 s^{2}+76.79 s+412.5
\end{align*}
$$

It has been revealed that this $C_{2}(s)$ is exactly obtained by setting $Q_{2}(s)=0$ in all stabilizing 2DOF controllers $\mathcal{T}\left(\mathcal{P}_{\alpha}\right)$ in (3) under the factorization in (6).

By fixing $C_{2}(s)$, we are ready to design $C_{1}(s)$. Parameterizing the set of all 2DOF controllers in (3) with $C_{2}(s)$ in (5) as an initial stabilizing controller and $Q_{2}(s)=0$ yields

$$
\mathcal{T}_{1}\left(P_{\alpha}\right):=\left\{\boldsymbol{C}_{\alpha}(s)=\left[\begin{array}{ll}
\frac{Q_{1}(s)}{X(s)} & \frac{Y(s)}{X(s)}
\end{array}\right]: Q_{1}(s) \text { is stable }\right\} .
$$

Let $r(t)=\sigma(t)$ and denote the error signal $e(t)=r(t)- z_{\alpha}(t)$. It follows from the problem formulation that the remaining optimization problem is as follows.

$$
\begin{array}{ll}
\operatorname{minimize} \boldsymbol{C}_{\alpha}(s) & \operatorname{deg} \boldsymbol{C}_{\alpha}(s) \\
\text { subject to } & e(t) \geq 0 \\
& \boldsymbol{C}_{\alpha}(s) \in \mathcal{T}_{1}\left(P_{\alpha}\right)
\end{array}
$$

Note that $G(s)$ from $r(t)$ to $z_{\alpha}(t)$ in (4) is given by

$$
\begin{align*}
G(s) & =N(s) Q_{1}(s)=\frac{b(s)}{f(s)} Q_{1}(s) \\
& =\frac{-0.5125\left(s^{2}+45.915\right)}{(s+11.3 \pm j 7.47)(s+1.24 \pm j 6.08)} Q_{1}(s) \tag{7}
\end{align*}
$$

and $C_{1}(s)$ is given by
$C_{1}(s)=\frac{h(s)}{p(s)} Q_{1}(s)=\frac{(s+12.06)(s+1.77 \pm j 5.58)}{(s+41.78)(s-0.57 \pm j 7.05)} Q_{1}(s)$.

The locations of poles of $G(s)$ in (7) have significant impact on its step response. Generally speaking, the poles which are closed to imaginary axis will be the ones dominating the step response. Moreover, if all the poles are real and stable, the step response can avoid an overshoot. Compared with the conjugate poles of $G(s)$ at $-11.3 \pm j 7.47$, the conjugate poles of $G(s)$ at $-1.24 \pm j 6.08$ are dominant poles. This pair of dominant poles possesses a large imaginary part causing
oscillation as well as potential overshooting phenomena. Therefore, the idea of designing a nonovershooting step response of $G(s)$ is as follows. The first is to cancel the pair of dominant poles using zeros of $Q_{1}(s)$. The second is to assign new poles to $G(s)$ to negative real numbers as much as possible using poles of $Q_{1}(s)$. The last is to determine the gain of $Q_{1}(s)$ by the DC gain $G(0)=1$ for the purpose of exactly tracking $\sigma(t)$ with zero steady-state error.

Since $C_{2}(s)$ is fixed and $\operatorname{deg} C_{2}(s)=3$, it holds that $\operatorname{deg} \boldsymbol{C}_{\alpha}(s) \geq 3$. By fully analyzing the poles and zeros of $G(s)$ in (7) and $C_{1}(s)$ in (8), one can show that the optimal value $\operatorname{deg} \boldsymbol{C}_{\alpha}(s)=3$ is impossible in our set-up and deg $\boldsymbol{C}_{\alpha}(s)$ should be at least 4 . We provide a set of optimal solutions of fourth-order $\boldsymbol{C}_{\alpha}(s)=\left[\begin{array}{ll}C_{1}(s) & C_{2}(s)\end{array}\right]$ where $C_{2}(s)$ is given in (5) and $C_{1}(s)$ is given by

$$
\begin{equation*}
C_{1}(s)=K \frac{(s+1.77 \pm j 5.58)(s+1.24 \pm j 6.08)}{(s+p)(s+41.78)(s-0.57 \pm j 7.05)} \tag{9}
\end{equation*}
$$

with a configurable scalar $p \in(0,2.08]$ and $K$ is a constant determined by $G(0)=1$. For all $p \in(0,2.08]$, one can prove that the step response of $G(s)$ in (7) is free of overshoots. Moreover, this design procedure also works for other choices of $\alpha \in[0.9,2.5]$ and will produce fourth-order optimal 2DOF controllers with a certain range of $p$. The detailed proof will be available in a longer version of this paper.

## B. Experimental results

The effectiveness of the proposed 2DOF controllers is demonstrated for the USUAL inverted pendulum in Fig. 3 for all the sensor locations $\alpha \in[0.9,2.5]$. During the initialization process of the experiment, we first rotate the pendulum anticlockwise from the downward mode to approach the upward mode. The 2DOF controller will be enabled when the pendulum angle reaches $\theta(t)=-0.04 \mathrm{rad}$ exactly. Then we set a reference signal with step time at 15 s to make the tracking of $z_{\alpha}(t)$ activated. Due to the restricted travel of the cart in the real system, we scale down the reference signal $r(t)$ from $\sigma(t)$ to $0.2 \sigma(t)$.

![](https://cdn.mathpix.com/cropped/cfa53b96-c79f-4c5c-b628-d83ae7cd91ce-4.jpg?height=499&width=652&top_left_y=1800&top_left_x=1185)
Fig. 3. The real USUAL inverted pendulum

A representative experimental result is illustrated by Fig. 4. In this experiment, the USUAL inverted pendulum is controlled by $\boldsymbol{C}_{\alpha}(s)$ with $C_{1}(s)$ in (9) with $p=2, K=-188$
and $C_{2}(s)$ in (5). The red circles shown in the above figure represent the moment when $\boldsymbol{C}_{\alpha}(s)$ is enabled. The result indicates that the output $z_{\alpha}(t)$ indeed tracks the reference input whose step time is $t=15 \mathrm{~s}$ and achieves 0.2 m within around two seconds. Since $z_{\alpha}(t)=x(t)-\alpha L \sin \theta(t)$ and the only equilibrium point suggests $\theta(t)=0, \dot{\theta}(t)=0$, the pendulum angle $\theta(t)$ is forced around zero as well as the cart position $x(t)$ is kept around 0.2 m . When $t \geq 17 \mathrm{~s}$, both $\theta(t)$ and $x(t)$ exhibit small oscillation influenced by ubiquitous uncertainties and perturbations. The reader may suspect that the existing oscillation in $z_{\alpha}(t)$ violates the nonovershooting requirement. We refer the reader to the stabilization experimental result in [4], where a similar level of oscillation exists relative to our result. This comparison demonstrates that the existing oscillation in $z_{\alpha}(t)$ is acceptable and the oscillation is indeed independent of the issue of overshoots. We conclude that $z_{\alpha}(t)$ is free of overshoots and $\boldsymbol{C}_{\alpha}(s)$ is shown to be effective for the USUAL inverted pendulum.

![](https://cdn.mathpix.com/cropped/cfa53b96-c79f-4c5c-b628-d83ae7cd91ce-5.jpg?height=1215&width=837&top_left_y=1065&top_left_x=208)
Fig. 4. Experimental results of the output $z_{\alpha}(t)$, the pendulum angle $\theta(t)$, the cart position $x(t)$ when the reference input $r(t)=0.2 \sigma(t)$, the sensor location $\alpha=1.5$ and the designed scalar $p=2$. The red circles represent the moment when $\boldsymbol{C}_{\alpha}(s)$ is enabled

## IV. CONCLUSION

In this paper, we present a 2DOF control problem of the USUAL inverted pendulum. The problem involves three requirements of the closed-loop system, namely, a nonovershooting step response, the largest robust stability margin and the minimum order of a 2DOF controller. We then study the issue of controller design and the effectiveness of the proposed controller is validated through experiments. To the best of our knowledge, this is the first successful experimental study on the nonovershooting regulation of an inverted pendulum using a single position sensor measurement.

## ACKNOWLEDGMENT

The authors would like to thank Dr. Sei Zhen Khong for useful discussions.

## References

[1] L. Qiu and K. Zhou, Introduction to Feedback Control. Upper Saddle River, NJ: Prentice-Hall, 2009.
[2] Y. Leong and J. Doyle, "Understanding robust control theory via stick balancing," in Proc. 55th IEEE Conf. Decision and Control, Las Vegas, NV, 2016, pp. 1508-1514.
[3] J. Doyle, Y. Nakahira, Y. P. Leong, E. Jenson, A. Dai, D. Ho, and N. Matni, "Teaching control theory in high school," in Proc. 55th IEEE Conf. Decision and Control, Las Vegas, NV, 2016, pp. 59255945.
[4] C. Chen, D. Zhao, and L. Qiu, "Control of an under-sensed and underactuated linear inverted pendulum," in Proc. 57th Annu. Conf. SICE, Nara, Japan, 2018, pp. 1301-1306.
[5] G. Deodhare and M. Vidyasagar, "Design of non-overshooting feedback control systems," in Proc. 29th IEEE Conf. Decision and Control, 1990, pp. 1827-1834.
[6] S. F. Phillips and D. E. Seborg, "Conditions that guarantee no overshoot for linear systems," International Journal of Control, vol. 47, no. 4, pp. 1043-1059, 1988.
[7] S. Jayasuriya and J.-W. Song, "On the synthesis of compensators for non-overshooting step response," in Proc. 1992 American Control Conf., 1992, pp. 683-684.
[8] S.-K. Lin and C.-J. Fang, "Nonovershooting and monotone nondecreasing step responses of a third-order SISO linear system," IEEE Transactions on Automatic Control, vol. 42, no. 9, pp. 1299-1303, 1997.
[9] S. Darbha and S. P. Bhattacharyya, "On the synthesis of controllers for a non-overshooting step response," IEEE Transactions on Automatic Control, vol. 48, no. 5, pp. 797-800, 2003.
[10] R. Schmid and L. Ntogramatzidis, "A unified method for the design of nonovershooting linear multivariable state-feedback tracking controllers," Automatica, vol. 46, no. 2, pp. 312-321, 2010.
[11] M. Krstic and M. Bement, "Nonovershooting control of strict-feedback nonlinear systems," IEEE Transactions on Automatic Control, vol. 51, no. 12, pp. 1938-1943, 2006.
[12] M. Vidyasagar, Control System Synthesis: A Factorization Approach. Cambridge, MA: MIT Press, 1985.
[13] C. Desoer and C. Gustafson, "Algebraic theory of linear multivariable feedback systems," IEEE Transactions on Automatic Control, vol. 29, no. 10, pp. 909-917, 1984.
[14] D. Youla and J. Bongiorno, "A feedback theory of two-degree-offreedom optimal Wiener-Hopf design," IEEE Transactions on Automatic Control, vol. 30, no. 7, pp. 652-665, 1985.
[15] K. J. Aström and R. M. Murray, Feedback Systems: An Introduction for Scientists and Engineers. Princeton, NJ: Princeton University Press, 2010.
[16] D. Zhao, C. Chen, S. Z. Khong, and L. Qiu, "Robust control against uncertainty quartet: A polynomial approach," in Uncertainty in Complex Networked Systems, T. Başar, Ed. Cham, Switzerland: Birkhäuser, 2018, pp. 149-178.
[17] B. A. Francis, A Course in $\mathcal{H}_{\infty}$ Control Theory. Berlin, Germany: Springer-Verlag, 1987.
[18] G. Vinnicombe, Uncertainty and Feedback: $\mathcal{H}_{\infty}$ Loop-shaping and the $\nu$-gap Metric. London, UK: Imperial College Press, 2001.


[^0]:    *This work was supported by Guangdong Science and Technology Department, China, under the Grant No. 2019B010117002.

    Chao Chen, Di Zhao and Li Qiu are with the Department of Electronic and Computer Engineering, The Hong Kong University of Science and Technology, Clear Water Bay, Kowloon, Hong Kong, China \& The Hong Kong University of Science and Technology Shenzhen Research Institute, Shenzhen 518063, China. E-mails: cchenap@connect.ust.hk, dzhaoaa@connect.ust.hk, eeqiu@ust.hk.

