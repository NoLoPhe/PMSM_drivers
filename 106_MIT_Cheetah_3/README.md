Link: https://ieeexplore.ieee.org/document/7827048
Link: https://sci-hub.mksa.top/10.1109/TRO.2016.2640183

# Proprioceptive Actuator Design in the MIT Cheetah: Impact Mitigation and High-Bandwidth Physical Interaction for Dynamic Legged Robots

Patrick M. Wensing, Member, IEEE, Albert Wang, Student Member, IEEE, Sangok Seok, Member, IEEE,
David Otten, Jeffrey Lang, Fellow, IEEE, and Sangbae Kim, Member, IEEE

Manuscript received September 22, 2015; revised June 17, 2016 and September 15, 2016; accepted November 12, 2016.
P. M. Wensing, A. Wang, and S. Kim are with the Department of Mechanical Engineering, Massachusetts Institute of Technology, Cambridge, MA 02139 USA (e-mail: pwensing@mit.edu; adwang@mit.edu; sangbae@mit.edu).
S. Seok is with Naver Corporation, Seongnam 13561, South Korea (e-mail: sangok.seok@navercorp.com).
D. Otten and J. Lang are with the Department of Electrical Engineering and Computer Science, Massachusetts Institute of Technology, Cambridge, MA 02139 USA (e-mail: otten@MIT.EDU; lang@mit.edu).
Color versions of one or more of the figures in this paper are available online at http://ieeexplore.ieee.org.
Digital Object Identifier 10.1109/TRO.2016.2640183

**Abstract** — Designing an actuator system for highly dynamic legged robots has been one of the grand challenges in robotics research. Conventional actuators for manufacturing applications have difficulty satisfying design requirements for high-speed locomotion, such as the need for high torque density and the ability to manage dynamic physical interactions. To address this challenge, this paper suggests a proprioceptive actuation paradigm that enables highly dynamic performance in legged machines. Proprioceptive actuation uses collocated force control at the joints to effectively control contact interactions at the feet under dynamic conditions. Modal analysis of a reduced leg model and dimensional analysis of DC motors address the main principles for implementation of this paradigm. In the realm of legged machines, this paradigm provides a unique combination of high torque density, high-bandwidth force control, and the ability to mitigate impacts through backdrivability. We introduce a new metric named the “impact mitigation factor” (IMF) to quantify backdrivability at impact, which enables design comparison across a wide class of robots. The MIT Cheetah leg is presented, and is shown to have an IMF that is comparable to other quadrupeds with series springs to handle impact. The design enables the Cheetah to control contact forces during dynamic bounding, with contact times down to 85 ms and peak forces over 450 N. The unique capabilities of the MIT Cheetah, achieving impact-robust force-controlled operation in high-speed three-dimensional running and jumping, suggest wider implementation of this holistic actuation approach.

**Index Terms** — Actuators, design engineering, dynamics, legged locomotion.

## I. INTRODUCTION

PHYSICAL interactions with the environment play a crucial role in many emerging applications of robotics. Whether in legged locomotion or disaster response, the need to control and exploit interaction with the environment introduces unique challenges to actuator design. As we focus on these applications, it is imperative to shift our actuator paradigms away from technologies that have been optimized for the assembly line floor. Most manufacturing robots have been designed to perform rapid and accurate pick-and-place position control sequences in wellknown environments. Future mobile robots, however, must be capable to manage broader dynamic and force-critical interactions in unstructured environments. The actuator paradigm presented in this paper offers an approach for electromagnetic (EM) actuators to address these needs. 

The design of actuators to manage physical interactions is particularly challenging for application to dynamic legged locomotion. Legged locomotion involves repeating dynamic events such as impact, high-force interaction with uncertain terrain, and rapid leg swing. These events place unique and often conflicting requirements on the design of EM actuators. Design processes for EM components are traditionally governed by performance metrics, such as the motor constant, torque constant, peak torque, continuous torque, thermal capacity, and others [1]. Beyond the EM components, the transmission (speed reducer) is a critical additional component that affects the control bandwidth and ability to handle collisions with the environment [2]. Multiobjective design tradeoffs thus need to be weighed in the selections of both the EM components and transmissions. Three common EM actuator concepts that manage these tradeoffs for robotics are shown in Fig. 1 and are discussed throughout the paper.

We propose to address this intricate actuator design problem through a new paradigm for EM actuators in legged machines called proprioceptive actuation. As a key feature, this allows “proprioceptive” foot force control through internal torque control. This can be accomplished without exteroceptive sensory feedback that is known to cause noncollocated sensing problems upon collision which result in contact instability [3]. The paradigm of proprioceptive actuation has had success in lowforce haptic devices such as the PHANTOM [4], but previously has not been employed for high-force applications such as legged locomotion. Overall, the paradigm eliminates the need for physical springs, stiffness modulating mechanisms, and force/torque sensors, and enables high-bandwidth force control without contact force feedback.

Proprioceptive actuation is a unique paradigm to address many of the design challenges facing legged locomotion. These challenges are presented in Section II with a summary of the previous approaches developed to handle them. Through modal analysis of a simple prototype of a legged system, and dimensional analysis of motor performance across scale, Section III shows that the proprioceptive paradigm is uniquely situated to manage high-force, high-bandwidth contact interactions imperative to high-speed locomotion. We view handling impact as the most difficult of these contact interactions, and propose a new metric in Section III-B called the impact mitigation factor (IMF). The IMF quantifies the normalized inertial impedance of a floating-body robot, capturing the effects of actuator design to reduce impulsive forces at impact.

The MIT Cheetah design, presented in Section IV, embodies the principles of proprioceptive actuation. By minimizing the overall impedance of the actuator, the design is able to mitigate impacts comparably to existing quadrupeds with series elastic actuators (SEAs), while maintaining high-bandwidth open-loop force control (see Section V). A discussion in Section VI details the high-level benefits on this paradigm which embraces a notion of “less is more” in mechanical design. We believe this approach provides a robust and practical new paradigm for legged machines.

## II. CHALLENGES IN ACTUATOR DESIGN FOR HIGH-SPEED LEGGED ROBOTS

One of the main challenges in actuator design is to identify critical metrics for the target application and tradeoffs between the metrics across design parameters. For dynamic legged locomotion, we identified the following key actuator characteristics:
  - 1. high mass-specific torque (torque density);
  - 2. efficiency; and
  - 3. impact mitigation capability.
The following sections describe these three aspects in more
detail.

### A. Torque Density

Achieving high torque with a minimum actuator mass is critical in running robots. This is due to in part to the high ground reaction forces (GRFs) required to propel the body. The maximum normal GRF on each leg is about three times the bodyweight in a human running at 4.5 m/s [6]. In data for a dog galloping at 9 m/s [7], it reaches 2.6 times the bodyweight during a ground phase of approximately 70 ms (20% of gait period). While long legs provide a large workspace, they intensify the torque requirements to produce these forces. To meet GRF requirements at the foot, large moment arms dictate high joint torques. For example, the maximum torque at the knee required for a 75 kg human to jump is around 200 N·m [8]. This is remarkable considering that the torque from a 1270 kg Toyota Corolla is 170 N·m at 4000 r/min [9].

Simply employing large motors to meet torque requirements is not viable in mobile robot design. Higher actuator masses contribute to GRF requirements, and thus increase torque requirements. This relationship highlights the importance of maximizing torque density for actuators in legged machines.

The power of the motor often becomes a major metric in designing motors for traditional applications. However, the high power achieved by high-speed continuous operation does not apply to legged machines, where operation is mostly intermittent and varying directions continuously. In fact, the power density of EM actuators themselves (continuous up to 7 kW/kg [10], 3–5 kW/kg [11]) can significantly exceed that of biological muscle (Max. 0.3 kW/kg) [12], and of most manufacturerrecommended operating specifications for commercial motors.

### B. Energy Efficiency

Energy efficiency is a critical metric for legged machines. Locomotion is a typically energy dissipative process and the loss mechanisms are categorized into three major modes: Joule heating, transmission, and interaction losses [13]. We previously described design principles to minimize these losses for highly efficient legged robots. Their implementation, in the design of the MIT Cheetah, has led to a machine that rivals the efficiency of animals [13]. However, this design was not explicitly optimized for efficiency.

Transmission selections, in particular, have coupled implications for Joule heating and interaction losses. Low gear ratios will reduce the reflected inertia but can cost higher energy in generating torque via Joule heating. On the other hand, if the gear ratios used in a robot are too high for its application, they may improve energetics in theory but can break a leg upon contact or even could prevent a desired dynamic motion due to excessive mechanical impedance. This tradeoff should further be considered based on the robot’s functional requirements such as payload, required travel distance per charge, and travel speed. For instance, a load-carrying robot walking at slow speeds could energetically benefit from a higher gear ratio in comparison to an agile robot running at high speeds. Gear efficiencies and frictional properties play a major role in balancing energetic tradeoffs. Yet, while there exist many methods to identify gear energetics of a fabricated design [14], accurately modeling gear energetics a priori for design optimization is complicated by a variety of factors [15]. Still, whereas energy efficiency is a metric we should seek to maximize, impact mitigation is an essential requirement.

C. Impact Mitigation

While satisfying the torque and power requirements, the leg should be able to mitigate impact forces upon collision. There are a number of approaches in the literature which may use active control or passive mechanical dynamics to minimize collisional forces surrounding an impact event.

Researchers have developed several approaches that employ impedance control with high-gear-ratio actuators to achieve force reduction after impact. Apparent impedance can be modulated by implementing impedance control [16] with torque sensors at each joint [17], [18]. However, this control approach used with high-gear-ratio actuators has not demonstrated robustness to impacts or high-bandwidth force control capabilities for dynamic legged locomotion. Previous work has addressed humanrobot impacts in industrial settings with the DLR Lightweight
Robot III [19]. Initial impact events are detected, and effective control postimpact is shown to reduce overall collision severity. As a main difference, proprioceptive actuation provides a mechanical approach to minimize the severity of initial and postimpact forces, which can otherwise be detrimental in high-speed collisions without sufficient compliance.

The series elastic actuation (SEA) paradigm [20]–[22] has been employed in legged machines to reduce initial rigid-body impacts by purposefully adding mechanical elasticity in series with an actuator. More recently, numerous designs have been presented for variable stiffness actuators (VSAs) to mitigate impacts. Stiffness modulation in VSAs has been achieved through a variety of mechanisms. Designs may strategically load nonlinear springs [23] to modulate stiffness or may use linear springs by stretching them in a nonlinear way [24]. Vanderborght et al. [25] provide a thorough review and classification of the many ways to provide and modulate stiffness in VSAs. Other SEA designs, such as in the quadruped StarlETH, have demonstrated successful execution of controlled variable leg impedance using fixed stiffness joint SEAs [26]. While SEAs offer a great potential actuator solution for legged robots, their force bandwidth can suffer in comparison to designs without added compliance. Section III will quantify this effect in a simple leg model.

III. PROPRIOCEPTIVE FORCE CONTROL ACTUATION

A. Impact Force Analysis in a Simplified Leg Model

This section studies how the design parameters of an actuation system affect its impact dynamics and force control. A simple model, shown in Fig. 2, was developed to capture essential design parameters for a legged system. The model consists of a body mass mb at height yb . Forces delivered to the mass are modulated by an actuated rack and pinion. A pinion of radius r is driven by an ideal actuator with output torque τ . An inertia I represents the total rotational inertia of the actuator including gear transmission. The rack, with mass m , abstracts the entire leg and interfaces with the ground through a Hookean spring with stiffness ki. This stiffness represents a lumped stiffness of the ground, foot covering, and any residual structural stiffness of the leg. We assume that this interface spring has no preload. This assumption is captured in the model by placing the origin in the vertical direction at a height which corresponds to the spring’s rest length at impact.

Let q = [yb , θ] T , the dynamics of the leg model follow

>       H ¨q + K q + g = ST τ         (1)

from standard Lagrangian methods, with

>       H = mb + m −rm−rm mr2 + I
>       g = (mb + m )g −mgr           (2)
>       K = ki −rki −rki r2ki 
>       S = 0 1 .                   (3)

A modal analysis of the system is tractable, providing insight into both the impact dynamics and the mechanical bandwidth of the leg. This simple linear model captures many features pertinent to the design of multi-DoF legs, such as the effects of reflected actuator inertia on the GRFs following impact. The GRF f(t) onto the leg is provided by the interface spring 

>       f(t) = −ki y (t) = −[ki − rki]   	 :=C    q(t). (4)

Let s be the Laplace variable, the Laplace transform of the GRF F(s)is related to the Laplace transform of the torque T (s) and initial conditions through

>       F(s) = C(s2H + K) 1 ST T (s) − g/s + H(sq0 + q˙ 0 ).(5)

To analyze properties of the GRF, drop-test conditions are assumed. The leg is assumed stationary with respect to the body at an impact with initial velocity y˙b,0 . As such, q0 = [0, 0] , and q˙ 0 =[˙yb,0 , 0]T . The Laplace transform of the GRF F(s) can then be found through algebraic expansion of (5) and is given as

>       F(s) = rkimb d(s) T (s) + ki(mbmr2 + Imb + Im ) d(s) −y˙b,0 + g s      (6)

where

>       d(s)=(mbI + mI + r2mbm ) s2 + ki(mbr2 + I).           (7)

This second-order system has a single natural frequency  ωn = ki/me , where me is the effective mass felt by the spring

>       me = mbI + mI + r2mbm mbr2 + I .        (8)

It is interesting to note that the reflected actuator inertia directly modulates the effective mass

>       lim I→0 me = m and lim I→∞ me = m + mb  (9)

where any increase in I leads to an increase in me

>       dmedI = r2m2b(mbr2 + I)2 > 0 .          (10)

Furthermore, assuming

>       J := ∂y ∂yb ∂y ∂θ = [1, −r]             (11)

as the Jacobian for leg mass, straightforward algebra verifies 

>       me = JH−1JT −1 .                       (12)

Thus, me is precisely a task-space (operational-space) inertia [27] measured at the foot prior to the interface spring. With this connection, Section III-B generalizes the analysis to higher DoF articulated mechanisms.

Returning to the system (6) and examining the impact force in more detail, we assume τ = 0 in order to isolate the passive mechanical properties of the mechanism. The inverse Laplace transform of (6) then provides 

>     f(t) = −y˙b,0 kime sin(ωn t) + meg(1 − cos(ωn t)).         (13)

Fig. 3 shows the dependence of the maximum impact force and mechanical bandwidth ωn on the leg design parameters I and m . Due to the dependence of impact force and bandwidth on me , a combination of low leg mass and low actuator inertia simultaneously maximizes open-loop force control bandwidth and minimizes impact force magnitudes. For high leg mass, the actuator inertia has comparatively less effect on these metrics.

This analysis was extended to the case of including series compliance into the pinion actuation, as shown in Fig. 2(b). The addition of compliance helps to soften impact, but is known to have a detrimental effect on closed-loop force control bandwidth. In terms of design, the requirements of a closed-loop controller to fight the natural dynamics of the system in highfrequency regimes can be minimized by designing to maximize the open-loop mechanical bandwidth of the mechanism. For this extended model, we approximate its mechanical bandwidth with its lowest natural frequency.

In this extended case, the system dynamics can again be placed in the form of (1). A series spring of stiffness ks is modeled between a motor angle θm and a spring output angle θ. The extended system thus has configuration q = [yb , θ, θm ]. The system kinetic T and potential V energies, respectively, take the form

>       T = 1 2 mb y˙ 2 b + 1 2 I ˙ θ2 m + 1 2 m y˙ 2             (14)
>       V = g mb yb + g m yl + 1 2 ki y2 + 1 2 ks (θm − θ) 2 .    (15)

A Lagrangian development can be followed to derive the dynamics, again resulting in a linear system (1).

Following this development, the GRF for the extended system again matches (5). The torque-to-force transfer function Hτ (s) satisfies

>       Hτ (s) = C(s2H + K) −1ST = ks rkimb αs4 + βs2 + γ         (16)

where

>       α = Ir2mbm                                                (17)
>       β = ks (Imb + Im + r2mbm ) + Ir2kimb                      (18)
>       γ = kski(mbr2 + I).                                       (19)

Letting j represent the imaginary variable, the four poles s = ±j ω1,2 of (16) provide the two natural frequencies of this system as

>       ω1,2 =  β ∓ β2 − 4αγ 2α .                               (20)

Graphs of these two frequencies are shown in Fig. 4. As intuition may suggest, ω1 → ωn as ks → ∞, where ωn is the natural frequency for the previous prototype with rigid transmission. At low transmission stiffnesses ks , the interface spring ki and leg mass m do not affect the mechanical bandwidth

>       ω˜1 :=  ks  Ir2mb I + r2mb ≈ ω1 .                       (21)

At high stiffnesses ks , a similar, yet less physically meaningful result holds for ω2

>       ω˜2 :=  ks  Ir2mbm r2mbm + I(m + mb ) ≈ ω2 .            (22)

Although it is possible to achieve a closed-loop force control bandwidth beyond the lowest natural frequency of the system, this process is sensitive to model-based information and requires additional actuation effort to fight the natural dynamics of the system. This is further complicated in applications of noncollocated force control which must compensate for the detailed dynamics of potentially many transmission elements between the sensor and the actuator.

Thus, we argue that the open-loop mechanical bandwidth of a mechanism ultimately limits the practical closed-loop bandwidth. Indeed, with the numbers used in the figure above ωn = 79.7 Hz, a value similar to that reported with our framework in Section V. The addition of a stiffness ks = 70 N·m/rad, similar to that in StarlETH [26], provides ω1 = 13.5 Hz, similar to the 9 Hz closed-loop bandwidth reported in their LQR-based SEA torque control.

Through this analysis, it follows that to minimize impact forces and maximize mechanical bandwidth, a design should be sought with minimal reflected actuator inertia, minimal leg mass, and minimal actuator compliance. Reflected inertia, in particular, has been shown to play an important role in the effective mass that governs collisional dynamics. The next section will address how to generalize and quantify these effects in more complete leg models.

### B. Impact Mitigation Factor

In more complex mechanisms, factors such as actuator placement and the structure of the leg articulation ultimately determine how reflected actuator inertias affect the backdrivability of the robot. While backdrivability includes both velocitydependent and inertia-dependent effects [2], inertia-dependent effects are much more difficult to shape through closed-loop impedance control [16], [28]. Thus, these passive inertial effects are purely dependent on the inherent design characteristics of the robot. Of these inherent characteristics, reflected actuator inertias play an important role. They directly determine the degree to which the body inertia contributes to impacts, as observed in the simple leg model in (9). As a result, the highest loads felt in the legs, gearboxes, and other transmission components are governed, in large part, by design decisions centered on reflected actuator inertia.

Previous work has addressed the role of the effective contact inertia in modeling impact events [29], which is used in Section III-B1. This previous work modeled the values of impact impulses under specific impact conditions, which may be of use in comparing designs for a specific robot. For unstructured environments where impact conditions may not be known, however, maximizing overall inertial backdrivability is imperative to mitigate impact forces. 

With these impact-related motivations, this section quantifies how effectively the free dynamics of the mechanism are at reducing impact impulses in a floating-body robot. To emphasize the importance of actuator design on impact dynamics, impulse reduction is evaluated through comparison to a design with worst case reflected inertia, one where all joints are rigidly locked. Through this approach, a new metric, called the impact mitigation factor (IMF), is introduced to quantify the normalized inertial backdrivability of the mechanism. Despite the importance of backdrivability to physical interaction, there are not yet meaningful metrics to compare backdrivability across machines and to include EM actuator properties in such a comparison. The new development discussed in Section III-B2 illuminates the effects of actuator design on previous impact analyses, while Section III-B3 formulates the new IMF metric to quantify inertial backdrivability across different robots.

1) Rigid-Body Impact Dynamics: Given a floating-body system, with base coordinates qb ∈ R6 and internal (joint) coordinates qj ∈ Rn its dynamics can be compactly described through

>       Hbb Hbj Hj b Hj j  ¨qb ¨qj + h(q, ˙q) = ST τ + JT f         (23)

where q = [qb , qj ] T ∈ Rn+6 , h(q, ˙q) ∈ Rn+6 includes the Coriolis, gravity, and spring-dependent terms, J ∈ Rm×(n+6) is a contact Jacobian, and f ∈ Rm represents the contact force. For point-foot contacts considered here, m = 3. In a case of series compliance at the joints, qj may include both joint and motor angles. In contrast to the previous section, we do not model any ground compliance, and assume that impact can be considered as an impulsive event. As a result, the analysis is idealized in comparison to that in the previous section.

Given a state q, ˙q just before a foot impact, the system hits the ground with a velocity v = J ˙q and experiences a contact impulse ρ ∈ R3 as given by [29] 

>       ρ = −Λv                                                     (24)

where Λ ∈ R3×3 is the operational-space inertia matrix (OSIM) [27], [30], [31] of the system felt at the contact and is given by

>       Λ = J H−1 JT −1 .                                          (25)

2) Actuator Effects on the OSIM: It is important to note how the reflected inertias of EM actuators affect the OSIM. Generally, the mass matrix H, as partitioned in (23), provides a kinetic energy metric

>       T = 1 2 ˙qT H ˙q                                            (26)

where T includes the kinetic energy of rigid-body links as well as rotor inertias and gears of EM actuators. Thus, the mass matrix can always be partitioned into a matrix that accounts for the kinetic energy of all rigid-body links Hr b and a matrix Hmot that accounts for the kinetic energy of the moving parts within link-mounted actuators (rotors, gears, etc.)

>       H = Hr b + Hmot .                                           (27)

Hmot is often approximated as [32]

>       Hmot = diag(01×6 , I1 ,...,In )                             (28)

where Ii represents the reflected rotational inertia on DoF i and scales as the square of its associated gear ratio. This approximation is only valid for actuators with large gear ratios, where the only significant kinetic energy of the motor elements is from rotational kinetic energy along their rotational axes. Regardless of this approximation, it can be seen from

>       Λ = J (Hr b + Hmot) −1 JT −1                               (29)

that any increase to the actuator inertia Hmot (in a positive semidefinite sense) necessarily gives rise to an increase in contact inertia Λ, generalizing the simplified result in (10).
