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

### C. Impact Mitigation

While satisfying the torque and power requirements, the leg should be able to mitigate impact forces upon collision. There are a number of approaches in the literature which may use active control or passive mechanical dynamics to minimize collisional forces surrounding an impact event.

Researchers have developed several approaches that employ impedance control with high-gear-ratio actuators to achieve force reduction after impact. Apparent impedance can be modulated by implementing impedance control [16] with torque sensors at each joint [17], [18]. However, this control approach used with high-gear-ratio actuators has not demonstrated robustness to impacts or high-bandwidth force control capabilities for dynamic legged locomotion. Previous work has addressed humanrobot impacts in industrial settings with the DLR Lightweight
Robot III [19]. Initial impact events are detected, and effective control postimpact is shown to reduce overall collision severity. As a main difference, proprioceptive actuation provides a mechanical approach to minimize the severity of initial and postimpact forces, which can otherwise be detrimental in high-speed collisions without sufficient compliance.

The series elastic actuation (SEA) paradigm [20]–[22] has been employed in legged machines to reduce initial rigid-body impacts by purposefully adding mechanical elasticity in series with an actuator. More recently, numerous designs have been presented for variable stiffness actuators (VSAs) to mitigate impacts. Stiffness modulation in VSAs has been achieved through a variety of mechanisms. Designs may strategically load nonlinear springs [23] to modulate stiffness or may use linear springs by stretching them in a nonlinear way [24]. Vanderborght et al. [25] provide a thorough review and classification of the many ways to provide and modulate stiffness in VSAs. Other SEA designs, such as in the quadruped StarlETH, have demonstrated successful execution of controlled variable leg impedance using fixed stiffness joint SEAs [26]. While SEAs offer a great potential actuator solution for legged robots, their force bandwidth can suffer in comparison to designs without added compliance. Section III will quantify this effect in a simple leg model.

## III. PROPRIOCEPTIVE FORCE CONTROL ACTUATION

### A. Impact Force Analysis in a Simplified Leg Model

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

  - 1. **Rigid-Body Impact Dynamics:** Given a floating-body system, with base coordinates qb ∈ R6 and internal (joint) coordinates qj ∈ Rn its dynamics can be compactly described through

>       Hbb Hbj Hj b Hj j  ¨qb ¨qj + h(q, ˙q) = ST τ + JT f         (23)

where q = [qb , qj ] T ∈ Rn+6 , h(q, ˙q) ∈ Rn+6 includes the Coriolis, gravity, and spring-dependent terms, J ∈ Rm×(n+6) is a contact Jacobian, and f ∈ Rm represents the contact force. For point-foot contacts considered here, m = 3. In a case of series compliance at the joints, qj may include both joint and motor angles. In contrast to the previous section, we do not model any ground compliance, and assume that impact can be considered as an impulsive event. As a result, the analysis is idealized in comparison to that in the previous section.

Given a state q, ˙q just before a foot impact, the system hits the ground with a velocity v = J ˙q and experiences a contact impulse ρ ∈ R3 as given by [29] 

>       ρ = −Λv                                                     (24)

where Λ ∈ R3×3 is the operational-space inertia matrix (OSIM) [27], [30], [31] of the system felt at the contact and is given by

>       Λ = J H−1 JT −1 .                                          (25)

  - 2) **Actuator Effects on the OSIM:** It is important to note how the reflected inertias of EM actuators affect the OSIM. Generally, the mass matrix H, as partitioned in (23), provides a kinetic energy metric

>       T = 1 2 ˙qT H ˙q                                            (26)

where T includes the kinetic energy of rigid-body links as well as rotor inertias and gears of EM actuators. Thus, the mass matrix can always be partitioned into a matrix that accounts for the kinetic energy of all rigid-body links Hr b and a matrix Hmot that accounts for the kinetic energy of the moving parts within link-mounted actuators (rotors, gears, etc.)

>       H = Hr b + Hmot .                                           (27)

Hmot is often approximated as [32]

>       Hmot = diag(01×6 , I1 ,...,In )                             (28)

where Ii represents the reflected rotational inertia on DoF i and scales as the square of its associated gear ratio. This approximation is only valid for actuators with large gear ratios, where the only significant kinetic energy of the motor elements is from rotational kinetic energy along their rotational axes. Regardless of this approximation, it can be seen from

>       Λ = J (Hr b + Hmot) −1 JT −1                               (29)

that any increase to the actuator inertia Hmot (in a positive semidefinite sense) necessarily gives rise to an increase in contact inertia Λ, generalizing the simplified result in (10).

In the worst case scenario, if reflected inertias become arbitrarily large on each joint, the system will roughly behave as if all the joints are locked. This worst case design can be used as a benchmark to evaluate the effects of reflected inertias in designs with the joints free to move. Partitioning the Jacobian as

>         J = Jb Jj                                               (30)

an impact velocity v would give rise to a contact impulse in a locked system ρL as

>        T −1 v. ρL = − Jb H−1 bb Jb                               (31)

With this insight, we define a locked contact inertia ΛL as
>        T −1 ΛL = Jb H−1  bb Jb.                                  (32) 

Intuitively, the apparent inertia of the system with joints locked should be greater that the inertia felt with the joints free to rotate. This can be shown more formally that ΛL ≥ Λ in a positive semidefinite sense. Proof is given in the Appendix. It is  important to note that ΛL is invariant with changes in gear ratio (assuming negligible change in overall mass).

  - 3) **Impact Mitigation—Development and Metric:** Given an impulse that is experienced in the locked system, it is natural to consider: How effectively are the free dynamics of the mechanism at mitigating this impulse? That is, assuming the same impact velocity, how would the impact in the free system compare to its nonbackdrivable equivalent? Given the locked impulse ρL , equal impact velocities v are achieved when the free system experiences an impulse

>       ρ = ΛΛ−1 L ρL . ΛΛ−1 L                                      (33)

Roughly, the term ΛΛ−1 characterizes how much inertia is felt Roughly, the term at impact in comparison to the overall (locked) system inertia. The reduction in impulse is then given by

>        ρL ρL − ρ = I − ΛΛ−1 L   :=Ξ                           (34)

where Ξ is introduced as an impact mitigation matrix, and its determinant

>       ξ := det(Ξ)  (35)

as the impact mitigation factor (IMF). This new factor has interesting properties that pertain to backdrivability. It is shown in the Appendix that 0 ≤ ξ ≤ 1. An IMF ξ = 1 corresponds to a system with perfect inertial backdrivability that eliminates all impact, whereas ξ → 0 as Λ → ΛL . Due to the normalization provided from the locked case (with roughly infinite reflected actuator inertia), the IMF is a nondimensional quantity that enables comparison across machines of different scale.

Furthermore, one may be interested in the capability of a design to mitigate impact in a particular direction, for instance, in the case of a hopping robot subject to predominantly vertical impacts. Given a direction x, we define the directional IMF (DIMF) ξx as
 
>       ξx = 1 −  1 T 2 x Λx 1 T 2 x ΛL x                           (36)

where ξx similarly inherits the property that 0 ≤ ξx ≤ 1 from the fact that ΛL ≥ Λ. This DIMF has a clear physical interpretation as well. The kinetic energy lost due to an impact with velocity v is 

>       1 vΛv . 2                                                   (37)

Thus, ξx quantifies the percent decrease in energetic losses afforded through the free dynamics of the mechanism when impacting in direction x.

  - 4) **Series Elasticity and the IMF:** When compliance is added to a stiff transmission model, portions of the actuator inertia are no longer inertially coupled to the foot. This inertial decoupling affects Λ and the associated impact impulse. The compliance, roughly, redistributes some of the impact impulse to a window of time following impact. Changes in the values of the stiffness modify how spread out the forces become, but do not change the initial impulse, which is governed by inertial properties alone. In this sense, adding compliance to a rigid transmission model will affect the IMF, while changes to the values of stiffness will not. Considering nonimpulsive impact events over a finite window of time, which should always be the case for collisions in reality, represents an important area for future extension of the IMF. Further details on the IMF for a system with SEAs are provided in comparison to the design of the MIT Cheetah leg in Section V.

### C. Geometric Considerations for EM Motors 
 
From the analysis of the previous section, we seek to determine principles for actuator selection that minimize actuator inertia while providing the high torques necessary for high-speed locomotion. Many design parameters of EM actuators contribute to overall performance. However, we consider gap radius to be a critical parameter for the purposes of maximizing torque density and providing transmission transparency. The gap radius of an EM actuator is the radius of the magnetic interface between the rotor and the stator. While gap radius is one of many possible condensed indicators of performance [33], it is directly related to torque density and torque per total inertia. Torque per total inertia, in turn, is directly related to the available bandwidth of the actuator.

 Ignoring edge effects, the axial length of the motor does not affect torque density and torque per inertia because increasing axial length is equivalent to adding identical motors on the same axle. In Fig. 5, Emoteq HT series motor characteristics are plotted against gap radius. Motors of the same gap radius and various lengths have overlapping values of torque density and torque per inertia, whereas gap radius directly affects those characteristics. For extreme geometries beyond these designs, edge effects degrade the metrics as length decreases.

 1) **Effects of Geometry on Motor Performance Metrics:** To understand the main effects of motor geometry on torque density and torque per inertia, we consider a class of designs under the following simplifying conditions. First, the radial thicknesses of the rotor and stator are assumed fixed. Second, the cylindrical geometries of the rotor and stator are approximated to first order as thin walls. As a result, the actuator mass is given by M = 2πrg l(ts ρs + tr ρr ) and rotor inertia by J = 2πltr ρr rg3, where rg is the gap radius, l is the axial length, and t∗ and ρ∗ are the radial thickness and density, respectively, of the stator and rotor. Third, current density is assumed constant [34]. Neglecting edge effects, this results in conditions of thermal and EM similarity, where the steady-state stator temperature and average shear stress σs on the rotor are constant across designs. The resulting torque from this stress τ = 2πrg2 lσs does depend on motor geometry.

Therefore, within this class of designs, we predict that torque density and acceleration capability (torque per rotor inertia) scale by the following relationships with rg , and we observe no effect from axial length
 
>       τ /M ∝ rg                               (38)
>       τ /J ∝ 1/rg .                           (39)

Another important characteristic is torque production efficiency, 2 is equivalent which is related to the motor constant KM . KM to the torque squared per unit ohmic power loss. Torque is generated from sheer stress σs ∝ IBn/A, where I is the motor current, B is the field strength of the magnets, n is the number of wires in a cross section perpendicular to the axis, and A ∝ rg is the area of the cross section. Using this relationship, torque production efficiency is given as

>       2 KM =  rg2 l2 B 2 n2 Aw τ2 Kt2 = 2 ∝ R I R ρlw  (40)

where Kt is the torque constant, R is the terminal resistance, Aw is the cross-sectional area of each wire, lw the total length of wire, and ρ is the resistivity of the wire material.

An additional set of assumptions is used to simplify this relationship. First, a wire length lw ∝ nl is used, which assumes all stator wire contributes to winding coils. Second, a fixed wire gauge is assumed, which implies that the number of wires in the cross section n ∝ rg . Thus, the relationship becomes

>       τ2 ∝ rg3 l . I2R  (41)

  - 2) **Comparison to Catalog Data:** Fig. 5 shows values for these metrics across data collected from Emoteq HT series motors. The max continuous torque as reported on data sheets was used to approximate conditions of EM similarity. Within this dataset, the torque density was found proportional to rg0.8 ; torque per rotor inertia proportional to rg−1.6 ; and torque production efficiency proportional to rg4.1 . These factors of proportionality, rg0.8 , rg−1.6 , and rg4.1 are relatively consistent with the proportions in our dimensional analysis rg1 , rg−1 , and rg3 from (38), (39), and (41), respectively.

The similarity is to be taken considering that the dimensions of these motors used in Fig. 5 do not exactly match our assumptions and, in particular, the assumption of constant stator and rotor thickness. In the Emoteq HT series motors, the stator and rotor thicknesses scale by ts ∝ rg0.8 and tr ∝ rg0.4 , respectively. The incorporation of these trends into the previous analysis would lead to a lower τ /J, lower τ /M , and higher Kt2 /R as observed in the motor catalog data. Changing rotor/stator thickness, however, has a nonlinear effect on the magnitude of the magnetic field at the rotor–stator air gap, and heavily influences stator winding design. As a result, the effects of these changes are difficult to model accurately in general. The remainder of the section returns to considerations of fixed rotor and stator thickness across designs.

  - 3) **Optimizing Geometry Under Design Constraints:** The theoretical motor performance metrics (38) and (39) can be helpful in making initial decisions related to actuator geometry while considering design constraints. Such considerations are unique relative to previous isometric scaling analyses that fixed geometry [34], [35]. Suppose a desired output torque τout and a fixed mass budget M for an actuator in the class considered previously. Under this mass budget, motor length is constrained to scale by l ∝ 1/rg , as shown in Fig. 6. To meet the output torque requirements across different designs, suppose that a gear train with ratio N is used.

As an initial approximation, assume that the gear box has negligible added mass, inertia, and friction torques from the gears themselves. The required gear ratio as a function of gap radius is then

>       τout ∝ 1/rg N= 2πrg2 lσs                    (42)

with a reflected rotor inertia through the gearbox of

>       Jref = Jr N 2 = 2πrg3 ltr ρr N 2 ∝ rg0      (43)

when the scalings for l and N are considered. While the total reflected rotor inertia and the total torque at the output shaft stay the same, increasing the gap radius lowers the required gear ratio. In summary, in a design space where the motor mass and the output torque requirement are held constant, and gearboxes are idealized, (43) shows that total reflected inertia remains constant across variations in gap radius.

Considering the nonidealized effects of gear transmissions, however, shows the benefits of high-gap-radius designs when the actuator mass budget is fixed. The results of constant output torque and constant reflected inertia only hold across different design geometries if the mass, friction, and inertia of the gears are ignored. Instead, if we assume that mass, friction, and gear inertia increase monotonically with gear ratio N , this implies in turn that τout will decrease and Jr will increase monotonically with N . Thus, these considerations favor a larger gap radius motor when mass budget is fixed. Such a design will have a smaller gear ratio, fewer gear stages, and less gear mass, resulting in less friction loss, higher torque density, higher bandwidth, and higher IMF.

Fig. 7. Different operational limits of a motor in torque–speed space. The green area represents the manufacturer-recommended operation that is bounded by the voltage limit and continuous torque determined by the thermal limit. The blue area shows the extended operating range of the motor by raising voltage and speed but maintaining current under the continuous torque limit. The motor may intermittently exceed the continuous torque limit and operate in the transient operation space shown in peach color. Beyond the saturation torque, parts of iron in the stator become magnetically saturated and the torque/current relationship becomes highly nonlinear as the torque constant decreases. This nonlinearity can be compensated for in control to provide accurate torque delivery. At the demagnetization torque, magnetic fields from the stator begin to demagnetize the rotor magnets, damaging the motor.

Therefore, in this design space, there is no tradeoff, and in its purest form, this analysis advocates for high-gap-radius directdrive robots [36]. However, the geometry required for direct drive to have enough torque is typically infeasible. For example, in order to design a direct-drive motor for the MIT Cheetah, one such motor would have been 76.2 cm in diameter with a 5-mm axial length. Considering these limitations, the optimal actuator for a given mass will thus consist of a motor with the largest gap radius as allowed by space and the smallest gear ratio as required by torque specifications.

  - 4) **Other Considerations**: For extreme geometries, edge effects begin to degrade the benefits of high-gap-radius designs. As a result, it is important to view this analysis as providing a guiding principle to focus design pressure for actuators in legged machines. Outside the class of motors considered, or at edges of the design space, detailed EM, winding, and manufacturing considerations should limit the degree to which gap-radius effects are regarded as dominant without further modeling.

In addition to motor geometry optimization, we can increase torque density by utilizing the intermittent torque of the motor, which is much higher than the continuous torque limit. These operating regimes are depicted in Fig. 7. Many robotic actuators operate under the continuous torque limit, although the duty factor of high torque usage in most legged robots is small. In legged locomotion, the continuous torque limit of the motor does not limit torque capability. Unlike most EM motor applications, the joint torque profiles in legged locomotion constantly fluctuate.

## IV. PROPRIOCEPTIVE ACTUATION IN THE MECHANICAL DESIGN OF THE MIT CHEETAH LEG

The MIT Cheetah is designed to emulate various locomotion capabilities of quadrupeds, such as walking, running, and jumping. Such behaviors involve repeated high impacts followed by short ground contact times. For example, the ground contact time of each leg during 6 m/s bounding is around 60 ms. In order to control the GRFs during such a short period of time, the system must have high-bandwidth force control and survive frequent high impact forces. The MIT Cheetah is the first embodiment of the proprioceptive actuation concept in legged locomotion.

Fig. 8 shows the detailed design of the leg module. The four hip modules of the robot are identical with minor differences in segment length ratios among the three links and between the front and rear legs. As discussed in Section III-C, it is critical to maximize the ratio of torque to inertia. We selected design parameters for the motor with the largest radius that could fit within a 12.5-cm-diameter space, which is constrained by the size of the MIT Cheetah. The first version of the MIT Cheetah, which trotted up to 6 m/s [13], used an Emoteq HT-5001 frameless motor. The second version of the MIT Cheetah, capable of bounding outdoors, uses custom motors designed by the MIT team [37]. The motor was specifically designed to maximize the saturation torque for a given mass, where the major design tradeoff is between ohmic loss and the saturation torque. The saturation torque density of the custom designed motor is around 27 N·m/kg (standalone without module or gearing), significantly higher than 9 N·m/kg in the Emoteq HT-5001. Given this high torque density, a 3 cm motor length is able to be used on each axis. The rotor inertia of the custom motor is three times that of the Emoteq HT-5001 and the attached leg inertia is slightly larger than the previous MIT cheetah.

Given the large torque capacity of the motor, we chose the gear ratio (1:5.8)[1] to meet the normal GRF generation requirements for running at a range of speeds up to 13.5 m/s. We chose a one-stage planetary gear train with four planet gears. Such a low gear ratio provides a higher IMF, beneficial for highly dynamic locomotion. Unlike traditional serial-link robots in which actuators are present at every joint, two actuators and the gear train are coaxially located at the hip of the leg to minimize the total moment of inertia, which allows for a compact and robust design. One of these actuators directly actuates the hip, while the other actuates the knee through a parallel linkage. The structure of the leg is also designed to minimize mass and leg inertia, and thus maximize the IMF. Bending stress is minimized in the leg structure by distributing tensile forces to the tendons.

[1] This gear ratio was also driven by practical considerations, as it is the largest single-stage ratio available for a standard planetary configuration with commodity English geartooth options.

Fig. 9. Visual description of the configuration dependence of the IMF, and main dynamics parameters for a leg of the MIT Cheetah. The dashed line represents an ideal DIMF of 1, while the blue represents the computed DIMF.

This method allows significantly lower inertia of the leg without compromising leg strength [38]. The shoulder module contains the motors (2.00 kg), 17-4 stainless steel gear trains (0.42 kg), and framing (1.23 kg) for a total mass of 3.65 kg. A leg with three ABS plastic links is attached to each module, with a combined mass of 850 g. The lightweight leg allows the center of mass to be located inside the shoulder module. As a result, a small moment of inertia of the leg allows for rapid movement and high-bandwidth force control.

## V. ASSESSMENT AND RESULTS

### A. Impact Mitigation Factor

In order to assess the backdrivability of the MIT Cheetah leg design, the IMF was computed and compared to other legged systems. As noted in Section III-B, the IMF is configuration dependent. This configuration dependence is shown visually in Fig. 9. The figure shows the DIMF ξx for directions x in the sagittal plane. For the Cheetah, the IMF correlates most strongly with knee angle, as shown in Fig. 10. Due to the large mass and inertia of the components within the Cheetah’s motors, (28) was not used for simplification in the calculation of the IMF. The motor rotors, for instance, have rotational inertias on out-ofplane principal axes that are of the same order of magnitude as for the leg links. These inertial contributions would be otherwise ignored through the use of (28). The IMF numbers account for the mass and inertias of the motor rotors and planetary gears, as estimated from CAD models. The rotor has inertia of 3.0279 × 10−4 kg·m2 along its rotational axis, with a 5.8:1 gear reduction, resulting in a reflected inertia of 0.0102 kg m2 . Other main parameters of the leg are given in Fig. 9 for reference.

The IMF of the Cheetah design was also compared with approximate StarlETH and HUBO Plus models. Rigid-body masses, CoM locations, and kinematics for StarlETH were taken from an available publication [26]. Previous work provided estimated rotational inertias in the sagittal plane [39]. Inertias in other directions were computed from an equidensity assumption on a bounding box estimated from graphical models. No reflected actuator inertia was modeled for StarlETH, as its SEAs decouple reflected inertia from the endpoint. The HUBO model was taken from a URDF of the HUBO Plus robot [40]. The dc actuators assumed at the joints had a rotor inertia of 3.33 × 10−6 kg·m2 with a 160:1 gear reduction, producing a total reflected inertia of 0.0852 kg·m2 . No reflected inertia effects of the gear reduction were modeled. IMFs were computed assuming impact in the center of the bottom of the foot.

Fig. 10 shows that the SEA-actuated StarlETH has the highest IMF over a range of knee angles. For these computations, the virtual leg in each system remained upright, while the knee angle was used to modulate virtual leg length. Low IMFs for small knee angles are in most part due to reduced backdrivability in the ẑ direction, as indicated by ξẑ . A similar degradation in ξx̂ is observed in StarlETH and Cheetah when the leg becomes fully collapsed at a knee angle of π rad. The IMF of HUBO experiences less peak-to-peak variability in IMF across knee angle. This difference can be attributed to the additional articulation in the HUBO hip and ankle which further prevent its upper-body mass from being felt at impact even for small knee angles.

To assess the relative contribution of reflected actuator inertia to the IMF of HUBO and Cheetah, hypothetical modifications of these machines were considered that included SEAs at the joints. Averaged over the configurations considered, the IMF of unmodified Cheetah was 90% that of the hypothetical SEA Cheetah. In comparison, the IMF of unmodified HUBO was 52% that of a hypothetical SEA HUBO. HUBO’s design does include significant actuator mass distally in the leg, which in part causes this difference.

### B. Step-Input Test on the MIT Cheetah Leg

To initially evaluate proprioceptive force control, the Cheetah robot leg was mounted inside an axial material testing device (Zwick Roell BX1-EZ005.A4K-000). Its stock force sensor was replaced with a six-axis force–torque sensor (ATI delta, SI-66060 calibration) which can measure up to 1980 N in the z-axis with 0.25 N resolution.

To identify the open-loop force control bandwidth of the leg, a 100 N step-input test was executed. A pure radial force was commanded and mapped to desired leg torques assuming static loading conditions. Desired joint torques were then realized using closed-loop motor current control with custom motor drivers and current control taking place at a rate of 20 KHz. The currents from these drivers were used to estimate the force at the foot, again under an assumption of static loading conditions. Fig. 11 shows the results of this test. In this figure, the black line is the commanded force, the red line is the estimated force from measured motor currents, and the blue line is the force measured by the external sensor. The discrepance between the estimated force and the exteroceptive sensed force highlights the presence of structural compliance in the leg. This structural compliance effectively decreases the bandwidth of the proprioceptive force control below that of the electrical dynamics. Assuming a second-order response, the response bandwidth of the leg system can be estimated based on the measured rising time, as shown in Table I.

Further details on similar experiments with the MIT Cheetah leg can be found in our previous publication [5]. This previous work demonstrated the capabilities of proprioceptive force control to realize a virtual stiffness at the foot. The next section goes beyond impedance control and demonstrates open-loop force control for more general force profiles.

### C. Proprioceptive Force Control Test

To investigate the performance of proprioceptive force control for dynamic locomotion, we performed in situ force control tests during unconstrained three-dimensional bounding. The robot bounded in free space using a control law from previous work [41]. The bounding controller would output desired forces fpro ∈ R3 for each stance leg, to be realized through proprioceptive actuation. These desired forces were mapped to joint torques through

>       τ d = S JT fpro .                      (44)

Here, J ∈ R matrix for the joints in the leg. The desired torque τ d ∈ R3 contains torques for the two proprioceptive actuators nominally in the sagittal x̂ẑ plane, as well as for a more traditional hip ab/ad actuator that nominally affects the lateral ŷ forces. Coordinate systems follow those shown in Fig. 10. The ATI delta force– torque sensor was embedded into the contact surface such that the front left leg of the robot could run on the sensor without interference to the gait. Fig. 12 shows a figure of this experimental setup.

Figs. 13–15 show the proprioceptive force control tracking in the sagittal plane where the proprioceptive actuators have force control authority. The force control figures show high-duty-ratio bounding (see Fig. 13) in comparison to medium and low-dutyratio bounding (see Figs. 14 and 15). Lower duty ratios require shorter contact times down to 85 ms in the most extreme case considered here. Again, we emphasize that aside from statics (44), no model-based information was used to translate desired force into commanded motor current across any of these results.

Table II quantifies the average tracking accuracy of the proprioceptive force control approach. Force control accuracy was evaluated following each impact transient, during the periods shown in gray in Figs. 13–15. These periods start when the measured vertical force crosses the proprioceptive force, and end when the proprioceptive force goes to zero. In general, higher duty ratio bounding results in lower average errors.

High-duty-ratio bounding in Fig. 13 shows one of the reasons to pursue proprioceptive force control. The vertical force of the first hop, in particular, shows a low-frequency (≈20 Hz) unforced oscillation in the vertical direction. This additional vibration mode is caused by unmodeled dynamics (compliance) of the leg structure and mechanisms. Future design efforts may be directed toward further increasing the stiffness-per-mass ratio of the legs in order to raise the natural frequency of the structural dynamics. It is these unmodeled transmission dynamics that limit the bandwidth of noncollocated force control schemes.

## VI. DISCUSSION

The proprioceptive actuation paradigm offers a number of qualitative benefits that extend beyond the analysis presented. While designing for high IMF has been motivated to reduce impact forces, high IMF designs indicate favorable actuator characteristics more broadly related to dynamic interactions with the environment. High IMF designs capture an ability to transparently control the interaction forces, and to protect both the environment and the robot in instances of collision. These benefits have practical importance, improving the lifetime of fragile transmission components, reducing the need for bulky structural designs, and reducing leg dynamics that complicate swing-leg control.

As shown in the previous section, the IMF is also tightly related to improved bandwidth in proprioceptive force control (closed-loop torque control with open-loop GRF control). In dynamic locomotion, foot force control in short stance times is critical for balance control. Noncollocated force control lacks robustness to unmodeled high-frequency dynamics incited around impact, causing contact instability [3]. The ability to stably make and break contact though proprioceptive force control motivates broader application of this paradigm for high-force interaction.

Increasing the torque density of an EM actuator itself is critical to achieve conflicting specifications for high IMF and high efficiency. Since the gear ratio is a critical design parameter in the tradeoff between the torque generation efficiency and the IMF, increasing torque density of the motor itself enables a designer to reduce the actuator gear ratios without compromising either of the aspects. Through exploiting this strategy, the MIT Cheetah has already achieved a high efficiency (total cost of transport of 0.5) rivaling animals in a similar scale [13], [42]. Further research to understand and quantify the exact energetic tradeoffs due to gear transmission selections may yet enable operation beyond the efficiency envelope of legged animals in nature.

While space and scope have prevented their description, these advances in actuator design for impact mitigation and highbandwidth physical interaction have been paramount toward this unique platform achieving recent feats such as running at up to 6 m/s and successfully jumping over obstacles up to 80% of the Cheetah’s nominal leg length [43]. Fig. 16 shows snapshots from an example of jumping motion.

## VII. CONCLUSION

This paper proposed a new actuator paradigm for high-speed running robots, provided analyses to demonstrate the central tenants of this paradigm, and presented experimental results of force tracking tests. While proprioceptive actuation was effectively implemented in early haptic devices, such as the PHANTOM, a prototype leg model illuminates its benefits to manage impact and open-loop force control for legged machines. Geometric scaling analysis indicates that increasing the gap radius benefits torque density, which plays a critical role in system energetics for locomotion. In order to quantify the backdrivability afforded through this paradigm, we have introduced the IMF that is applicable to robots driven by EM actuators with or without series elasticity at the joints. A successful implementation of the actuator design principles was shown to allow for high-force proprioception to deliver desired forces through contact with only motor current sensing. The experimental results show the promising performance of the proposed actuator design paradigm by evaluating the force production capabilities of a leg from the MIT Cheetah.

These results encourage broader adoption of proprioceptive actuators to manage physical interaction for emerging applications in robotics. From disaster response to assistance in the home, proprioceptive actuators may endow next-generation robots with the ability to stably make and break contact, while providing the high-bandwidth and force-critical capabilities to react in unstructured environments. Through further refinement directed toward these applications, we believe the proposed approach will enable the development of actuators that yet exceed the capabilities of biological muscles in every aspect.

## APPENDIX IMPACT MITIGATION FACTOR

In the main body of the text, the impact mitigation factor for a leg was introduced as

>       ξ = det I − ΛΛ−1 L                    (45)

where Λ ∈ Rm ×m is the contact-space inertia matrix of the leg, and ΛL ∈ Rm ×m is the contact-space inertia matrix if all of the joints were locked. In this Appendix, we will show that Λ ≤ ΛL and 0 ≤ ξ ≤ 1.

Using notation from the main body of the text

>            Hbb Hbj −1 JTb  Λ−1 = Jb Jj .      (46)

Then, defining A := (Hj j − Hj bH−1 bb Hbj )−1 it follows from the matrix inversion lemma that

>         −1 Hbb − Hbj H−1 −H−1 j j Hj b bb Hbj A −1 Λ =J JT −AHj b H−1 A bb (47)
>        −1  −1 −H−1 Hbb + H−1 bb Hbj AHj b Hbb bb Hbj A =J JT . −1 −AHj b Hbb A (48)
>       
Factorizing this matrix

>          T −1 −H−1 bb Hbj A −Hbb Hbj Λ−1 = Λ−1 + J JT L I I  (49)


where the second term on the right-hand side is positive semidefinite since A is a Schur complement of H. Thus 

>       Λ−1 ≥ Λ−1 L  and  Λ ≤ ΛL . −1  (50)

From this fact, it follows that 0 ≤ ΛL 2 ΛΛL 2 ≤ I. Thus

>       −1  −1  0 ≤ det I − ΛL 2 ΛΛL 2 ≤ 1, and  ≤ 1. 0 ≤ det I − ΛΛ−1 L  (51) (52)  


## REFERENCES

[1] D. C. Hanselman, Brushless Permanent Magnet Motor Design. Writers’ Collective, Cranston, RI, USA, 2003.

[2] W. Townsend and J. Salisbury, “Mechanical design for whole-arm manipulation,” in Robots and Biological Systems: Towards a New Bionics? (ser. NATO ASI Series), P. Dario, G. Sandini, and P. Aebischer, Eds. Berlin, Germany: Springer, 1993, vol. 102, pp. 153–164.

[3] S. D. Eppinger and W. P. Seering, “Three dynamic problems in robot force control,” in Proc. IEEE Int. Conf. Robot. Autom., 1989, pp. 392–397.

[4] T. H. Massie and J. K. Salisbury, “The PHANTOM haptic interface: A device for probing virtual objects,” in Proc. ASME Dyn. Syst. Control Division, vol. 55, no. 1, 1994, pp. 295–300.

[5] S. Seok, A. Wang, D. Otten, and S. Kim, “Actuator design for high force proprioceptive control in fast legged locomotion,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., Oct. 2012, pp. 1970–1975.

[6] M. F. Bobbert, M. R. Yeadon, and B. M. Nigg, “Mechanical analysis of the landing phase in heel-toe running,” J. Biomech., vol. 25, no. 3, pp. 223–234, 1992.

[7] R. M. Walter and D. R. Carrier, “Ground forces applied by galloping dogs,” J. Exp. Biol., vol. 210, no. 2, pp. 208–216, 2007.

[8] A. Tsiokanosa, E. Kellisb, A. Jamurtasa, and S. Kellisc, “The relationship between jumping performance and isokinetic strength of hip and knee extensors and ankle plantar flexors,” Isokinetics Exercise Sci., vol. 10, no. 2, pp. 107–115, 2002.

[9] “Toyota Corolla 2016 model brochure,” 2016. [Online]. Available: http://www.toyota.com/corolla/ebrochure/ 

[10] [Online]. Available: http://www.teslamotors.com. Accessed 2015.

[11] [Online]. Available: http://www.scorpionsystem.com. Accessed 2015.

[12] I. W. Hunter and S. Lafontaine, “A comparison of muscle with artificial actuators,” in Proc. IEEE Solid-State Sensor Actuator Workshop, 1992, pp. 178–185.

[13] S. Seok et al., “Design principles for energy-efficient legged locomotion and implementation on the MIT Cheetah robot,” IEEE/ASME Trans. Mechatronics, vol. 20, no. 3, pp. 1117–1129, Jun. 2015.

[14] G. Abba and P. Sardain, “Modeling of frictions in the transmission elements of a robot axis for its identification,” IFAC Proc. Volumes, vol. 38, no. 1, pp. 7–12, 2005.

[15] A. Wang and S. Kim, “Directional efficiency in geared transmissions: Characterization of backdrivability towards improved proprioceptive control,” in Proc. IEEE Int. Conf. Robot. Autom., May2015, pp. 1055–1062.

[16] N. Hogan, “Impedance control: An approach to manipulation: Part II– Implementation,” J. Dyn. Syst., Meas. Control, vol. 107, no. 1, pp. 8–16, 1985.

[17] G. Hirzinger et al., “DLR’s torque-controlled light weight robot III-are we reaching the technological limits now?” in Proc. IEEE Int. Conf. Robot. Autom., 2002, vol. 2, pp. 1710–1716.

[18] C. Ott, A. Albu-Schaffer, A. Kugi, and G. Hirzinger, “On the passivitybased impedance control of flexible joint robots,” IEEE Trans. Robot., vol. 24, no. 2, pp. 416–429, Apr. 2008.

[19] S. Haddadin, A. Albu-Schäffer, and G. Hirzinger, “Requirements for safe robots: Measurements, analysis and new insights,” Int. J. Robot. Res., vol. 28, no. 11/12, pp. 1507–1527, 2009.

[20] G. A. Pratt and M. M. Williamson, “Series elastic actuators,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., 1995, vol. 1, pp. 399–406.

[21] P. Gregorio, M. Ahmadi, and M. Buehler, “Experiments with an electrically actuated planar hopping robot,” in Experimental Robotics III. Berlin, Germany: Springer, 1994, pp. 267–281.

[22] M. H. Raibert, “Legged robots,” Commun. ACM, vol. 29, no. 6, pp. 499– 514, Jun. 1986.

[23] J. Hurst, J. Chestnutt, and A. Rizzi, “The actuator with mechanically adjustable series compliance,” IEEE Trans. Robot., vol. 26, no. 4, pp. 597– 606, Aug. 2010.

[24] I. Thorson and D. Caldwell, “A nonlinear series elastic actuator for highly dynamic motions,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., Sep. 2011, pp. 390–394.

[25] B. Vanderborght et al., “Variable impedance actuators: A review,” Robot. Auton. Syst., vol. 61, no. 12, pp. 1601–1614, 2013.

[26] M. Hutter, C. Gehring, M. Hopflinger, M. Blosch, and R. Siegwart, “Toward combining speed, efficiency, versatility, and robustness in an autonomous quadruped,” IEEE Trans. Robot., vol. 30, no. 6, pp. 1427–1440, Dec. 2014.

[27] O. Khatib, “A unified approach for motion and force control of robot manipulators: The operational space formulation,” IEEE J. Robot. Autom., vol. 3, no. 1, pp. 43–53, Feb. 1987.

[28] A. Albu-Schaffer, C. Ott, U. Frese, and G. Hirzinger, “Cartesian impedance control of redundant robots: Recent results with the DLRlight-weight-arms,” in Proc. IEEE Int. Conf. Robot. Autom., Sep. 2003, vol. 3, pp. 3704–3709.

[29] Y.-F. Zheng and H. Hemami, “Mathematical modeling of a robot collision with its environment,” J. Robot. Syst., vol. 2, no. 3, pp. 289–307, 1985.

[30] H. Asada, “Dynamic analysis and design of robot manipulators using inertia ellipsoids,” in Proc. IEEE Int. Conf. Robot. Autom., Mar. 1984, vol. 1, pp. 94–102.

[31] P. Wensing, R. Featherstone, and D. E. Orin, “A reduced-order recursive algorithm for the computation of the operational-space inertia matrix,” in Proc. IEEE Int. Conf. Robot. Autom., 2012, pp. 4911–4917.

[32] R. Featherstone, Rigid Body Dynamics Algorithms. New York, NY, USA: Springer, 2008.

[33] J. Sensinger, “Selecting motors for robots using biomimetic trajectories: Optimum benchmarks, windings, and other considerations,” in Proc. IEEE Int. Conf. Robot. Autom., 2010, pp. 4175–4181.

[34] B. Hannaford, P.-H. Marbot, P. Buttolo, M. Moreyra, and S. Venema, “Scaling of direct drive robot arms,” Int. J. Robot. Res., vol. 15, no. 5, pp. 459–472, 1996.

[35] R. S. Wallace and J. M. Selig, “Scaling direct drive robots,” in Proc. IEEE Int. Conf. Robot. Autom., May1995, vol. 3, pp. 2947–2954.

[36] H. Asada and K. Youcef-Toumi, Direct-drive Robots: Theory and Practice. Cambridge, MA, USA: MIT Press, 1987.

[37] N. Farve, “Design of a low-mass high-torque brushless motor for application in quadruped robotics,” Master’s thesis, Massachusetts Inst. Technol., Cambridge, MA, USA, 2012.

[38] A. Ananthanarayanan, M. Azadi, and S. Kim, “Towards a bio-inspired leg design for high-speed running,” Bioinspir. Biomim., vol. 7, no. 4, 2012, Art. no. 046005.

[39] M. Hutter, C. Remy, M. Hoepflinger, and R. Siegwart, “Efficient and versatile locomotion with highly compliant legs,” IEEE/ASME Trans. Mechatronics, vol. 18, no. 2, pp. 449–458, Apr. 2013.

[40] “OpenRAVE model of the KHR-4 “Jaemi” Hubo,” 2013. [Online]. Available: https://github.com/daslrobotics/openHubo 

[41] H.-W. Park, M. Y. Chuah, and S. Kim, “Quadruped bounding control with variable duty cycle via vertical impulse scaling,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., Sep. 2014, pp. 3245–3252.

[42] H.-W. Park, S. Park, and S. Kim, “Variable-speed quadrupedal bounding using impulse planning: Untethered high-speed 3D running of MIT Cheetah 2,” in Proc. IEEE Int. Conf. Robot. Autom., 2015, pp. 5163–5170.

 [43] H.-W. Park, P. Wensing, and S. Kim, “Online planning for autonomous running jumps over obstacles in high-speed quadrupeds,” in Proc. Robot., Sci. Syst., Rome, Italy, p. 9, Jul. 2015, [Online]. Available: http://www.roboticsproceedings.org/rss11/p47.html
 
**Patrick M. Wensing** (S’09–M’15) received the B.S., M.S., and Ph.D. degrees in electrical and computer engineering from The Ohio State University, Columbus, OH, USA, in 2009, 2013, and 2014, respectively. He is a Postdoctoral Associate in the Biomimetic Robotics Laboratory, Department of Mechanical Engineering, Massachusetts Institute of Technology (MIT), Cambridge, MA, USA. His research interests focus on the intersection of dynamics, optimization, and control as applied to agile, intelligent, and physically interactive robotic systems. 

Dr. Wensing received an NSF Graduate Research Fellowship for his dissertation research on balance control strategies for humanoid robots. At MIT, the results of his postdoctoral work on the MIT Cheetah 2 robot have received considerable publicity worldwide, with features from Time, Wired, and the Wall Street Journal. He served as the Co-Chair for the Robotics and Automation Society Student Activities Committee from 2012 to 2014.

**Albert Wang** (S’14) received the B.S. and M.S. degrees from the Department of Mechanical Engineering, Massachusetts Institute of Technology (MIT), Cambridge, MA, USA, in 2010 and 2012, respectively. He is currently working toward the Ph.D. degree at the Biomimetic Robotics Laboratory, MIT, working on the HERMES humanoid project.

 His research interests include dynamic legged locomotion, actuation systems, system architecture, and shared autonomy for teleoperation.

**Sangok Seok** (S’10–M’14) received the B.S. degree in mechanical engineering and M.S. degree in mechanical and aerospace engineering from the School of Mechanical and Aerospace Engineering, Seoul National University, Seoul, South Korea, in 2002 and 2004, respectively, and the Ph.D. degree in mechanical engineering from Massachusetts Institute of Technology, Cambridge, MA, USA, in 2014.

 He was with the Korean branch of National Instruments as an Applications Engineer and a Marketing Engineer from 2004 to 2009, and with the Mechatronics R&D Center, Samsung Electronics as a Principal Engineer from 2014 to 2015. He is currently a Leader of the Robotics Group, Naver Laboratories, Seongnam, South Korea. His research focuses on service robot platforms, which enable ambient intelligence, utilizing space and mobility.

Dr. Seok received the IEEE/ASME TRANSACTIONS ON MECHATRONICS Best Paper Award for 2016.

**David Otten** received the B.S. and S.M. degrees from Massachusetts Institute of Technology (MIT), Cambridge, MA, USA, in 1973 and 1974, respectively.

In 1974, he joined the MIT Electric Power Systems Engineering Laboratory as a Staff Engineer. Since 1984, he has been a Principal Research Engineer in the renamed Laboratory for Electromagnetic and Electronic System, MIT. His research interests include instrumentation, power electronics, and the micromouse robot contest.

**Jeffrey Lang** (F’98) received the B.S., M.S., and Ph.D. degrees from the Department of Electrical Engineering and Computer Science, Massachusetts Institute of Technology, Cambridge, MA, USA, in 1975, 1977, and 1980, respectively.

He joined the Faculty of MIT in 1980, where he is currently the Vitesse Professor of electrical engineering. He served as the Associate Director of the MIT Laboratory for Electromagnetic and Electronic Systems from 1991 to 2003 and currently serves as an Associate Director of the MIT Microsystems Technology Laboratories. He is a coauthor of Foundations of Analog and Digital Electronic Circuits (Morgan Kaufman), in 2005 and the Editor of, and a Contributor to, Multi-Wafer Rotating MEMS Machines: Turbines Generators and Engines (Springer), in 2010. He has written more than 280 papers and holds 26 patents in the areas of electromechanics, MEMS, power electronics, and applied control. His research and teaching interests focus on the analysis, design, and control of electromechanical systems with an emphasis on rotating machinery; micro/nanoscale (MEMS/NEMS) sensors, actuators and energy converters; flexible structures; and the dual use of electromechanical actuators as motion and force sensors.

Dr. Lang received four best-paper prizes from IEEE societies and two teaching awards from MIT. He is a former Hertz Foundation Fellow. He served as an Associate Editor of Sensors and Actuators between 1991 and 1994. He has also served as the General Co-Chair and Technical Co-Chair of the 1992 and 1993 IEEE Micro Electro Mechanical Systems Workshops, respectively, and the General Co-Chair of the 2013 PowerMEMS Conference.

**Sangbae Kim** (M’08) received the B.S. degree in mechanical engineering from Yonsei University, Seoul, South Korea, in 2001, and the M.Sc. and Ph.D. degrees in mechanical engineering from the Biomimetics Laboratory, Stanford University, Stanford, CA, USA, in 2004 and 2008, respectively.

 He is the Director of the Biomimetic Robotics Laboratory and an Associate Professor of mechanical engineering at Massachusetts Institute of Technology, Cambridge, MA, USA. His research interests focus on bioinspired robotic platform design by extracting principles from complex biological systems.

 Dr. Kim’s achievements on bioinspired robot development include the world’s first directional adhesive inspired from gecko lizards and a climbing robot, Stickybot, that utilizes the directional adhesives to climb smooth surfaces, featured in Time’s best inventions of 2006. The MIT Cheetah achieves stable outdoor running at an efficiency of animals, employing biomechanical principles from studies of the best runners in nature. This achievement was covered by more than 200 articles. He received the King-Sun Fu Memorial Best Transactions on Robotics Paper Award (2008), the DARPA Young Faculty Award (2013), and the National Science Foundation CAREER (2014) Award.
