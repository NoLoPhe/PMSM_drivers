Link: https://ieeexplore.ieee.org/document/7827048
Link: https://sci-hub.mksa.top/10.1109/TRO.2016.2640183

electromagnetic (EM)

PHYSICAL interactions with the environment play a crucial role in many emerging applications of robotics. Whether
in legged locomotion or disaster response, the need to control and exploit interaction with the environment introduces unique challenges to actuator design. As we focus on these applications, it is imperative to shift our actuator paradigms away from technologies that have been optimized for the assembly line floor. Most manufacturing robots have been designed to perform rapid and accurate pick-and-place position control sequences in wellknown environments. Future mobile robots, however, must be capable to manage broader dynamic and force-critical interactions in unstructured environments. The actuator paradigm presented in this paper offers an approach for electromagnetic (EM) actuators to address these needs.

The design of actuators to manage physical interactions is particularly challenging for application to dynamic legged locomotion. Legged locomotion involves repeating dynamic events such as impact, high-force interaction with uncertain terrain, and rapid leg swing. These events place unique and often conflicting requirements on the design of EM actuators. Design processes for EM components are traditionally governed by performance metrics, such as the motor constant, torque constant, peak torque, continuous torque, thermal capacity, and others [1]. Beyond the EM components, the transmission (speed reducer) is a critical additional component that affects the control bandwidth and ability to handle collisions with the environment [2]. Multiobjective design tradeoffs thus need to be weighed in the selections of both the EM components and transmissions. Three common EM actuator concepts that manage these tradeoffs for robotics are shown in Fig. 1 and are discussed throughout the paper.

We propose to address this intricate actuator design problem through a new paradigm for EM actuators in legged machines called proprioceptive actuation. As a key feature, this allows “proprioceptive” foot force control through internal torque control. This can be accomplished without exteroceptive sensory feedback that is known to cause noncollocated sensing problems upon collision which result in contact instability [3]. The paradigm of proprioceptive actuation has had success in lowforce haptic devices such as the PHANTOM [4], but previously has not been employed for high-force applications such as legged locomotion. Overall, the paradigm eliminates the need for physical springs, stiffness modulating mechanisms, and force/torque sensors, and enables high-bandwidth force control without contact force feedback.

Three different EM actuator concepts

- High-ratio geared motor with torque sensor
    - geared motor with force/ torque sensor
    - High gear Ratio Transmission
    - stiff sensor
    - leg
- SEA
    - Series Elastic Actuator
    - High gear Ratio Transmission
    - Spring, Encoder
    - Leg
- proprioceptive force control actuator (bộ truyền động điều khiển lực độc quyền)
    - High torque density motor
    - Low Gear Ratio Transmission
    - Low inertia leg

Fig. 1. Three different EM actuator concepts. (a) High-ratio geared motor with torque sensor, (b) SEA, and (c) proprioceptive force control actuator

Proprioceptive actuation is a unique paradigm to address many of the design challenges facing legged locomotion. These challenges are presented in Section II with a summary of the previous approaches developed to handle them. Through modal analysis of a simple prototype of a legged system, and dimensional analysis of motor performance across scale, Section III shows that the proprioceptive paradigm is uniquely situated to manage high-force, high-bandwidth contact interactions imperative to high-speed locomotion. We view handling impact as the most difficult of these contact interactions, and propose a new metric in Section III-B called the impact mitigation factor (IMF). The IMF quantifies the normalized inertial impedance of a floating-body robot, capturing the effects of actuator design to reduce impulsive forces at impact.

The MIT Cheetah design, presented in Section IV, embodies the principles of proprioceptive actuation. By minimizing the overall impedance of the actuator, the design is able to mitigate impacts comparably to existing quadrupeds with series elastic actuators (SEAs), while maintaining high-bandwidth open-loop force control (see Section V). A discussion in Section VI details the high-level benefits on this paradigm which embraces a notion of “less is more” in mechanical design. We believe this approach provides a robust and practical new paradigm for legged machines.
