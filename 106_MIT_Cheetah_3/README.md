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
  -1. high mass-specific torque (torque density);
  -2. efficiency; and
3) impact mitigation capability.
The following sections describe these three aspects in more
detail.
A. Torque Density
Achieving high torque with a minimum actuator mass is critical in running robots. This is due to in part to the high ground re
