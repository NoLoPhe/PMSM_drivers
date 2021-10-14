Link: https://arxiv.org/pdf/2110.00736.pdf
Link: https://www.arxiv-vanity.com/papers/2110.00736/

# Stanford Pupper: A Low-Cost Agile Quadruped Robot for Benchmarking and Education

Nathan Kau[1] and Stuart Bowers[2]

[1] Nathan Kau is with the Department of Mechanical Engineering, Stanford University, Stanford, United States fleecy@stanford.edu

[2] Stuart Bowers contributed as an independent researcher stuart.bowers@gmail.com

**ABSTRACT** — We present Stanford Pupper, an easily-replicated open source quadruped robot designed specifically as a benchmark platform for legged robotics research. The robot features torque-controllable brushless motors with high specific power that enable testing of impedance and torque-based machine learning and optimization control approaches. Pupper can be built from the ground up in under 8 hours for a total cost under $2000, with all components either easily purchased or 3D printed. To rigorously compare control approaches, we introduce two benchmarks, Sprint and Scramble with a leaderboard maintained by Stanford Student Robotics. These benchmarks test high-speed dynamic locomotion capability, and robustness to unstructured terrain. We provide a reference controller with dynamic, omnidirectional gaits that serves as a baseline for comparison. Reproducibility is demonstrated across multiple institutions with robots made independently. All material is available at https://stanfordstudentrobotics.org/quadruped-benchmark.

**INDEX** — Legged robotics, open source, robotics, quadruped, benchmark

## I. INTRODUCTION

Recent successes in learning and optimization-based controllers for legged locomotion have largely been isolated to a select set of complex and sophisticated hardware platforms such as Anymal [9] and MIT Cheetah 3 [2]. Given the complexity and slow iteration cycle of legged robots, we see little reproduction of legged robot research across different lab groups. One of the primary factors driving the complexity of these robots is that they depend on actuators that can precisely track torque commands. For learning approaches, torque control is important to ensure that the actuator models used in simulation closely parallel their physical counterparts. Similarly, optimization-based approaches often assume perfect torque control in the dynamics models, such as in the centroidal model-predictive-control method introduced in [4].

Additionally, new control methods are often presented without comparison to baseline or previously-developed controllers which makes it difficult to make direct comparison. More so, new methods are frequently introduced alongside new hardware, making it difficult to isolate the algorithmic contributions. This motivates the need for an open source legged robot that is 1) low-cost, 2) easily replicated, and 3) high performance with torque control.

To respond to this need, we present Stanford Pupper (Fig. 1) and an associated set of benchmarks. Pupper is an open source quadruped robot designed to make legged robotics research more accessible, cost-effective, and comparable across institutions. The robot is small, lightweight, and built with off-the-shelf components. The actuators are transparent and torque-controllable, which makes the robot amenable to sim-to-real applications. Most importantly, the robot is simple and easy to build, which makes it reliable and enables fast iteration. We also present a set of benchmarks for Pupper to easily compare performance of different control algorithms across different institutions. Before detailing Pupper, we show where Pupper stands with relation to previous work.

## II. RELATED WORK

Previous work has explored the development of legged robots that are low-cost, easily replicated, or high performance, but to the best of our knowledge, no work has yet integrated all three characteristics into one hardware platform. At the time of this writing, two of the primary robots used for state-of-the-art learning and optimization-based control are Anymal and the MIT Cheetah variants [11]. These robots use high-bandwidth, torque-controllable actuators to accurately track control policies. However, because of their complexity and their limited availability – Anymay is sold on a case-by-base basis, MIT Cheetah 3 is proprietary, and MIT Mini Cheetah is only partially open-source – few researchers outside of the robots’ original labs have experimented with them.

A quadruped robot that is low-cost and easily replicated is introduced by ROBEL [1], but this platform sacrifices performance to make it accessible. They also provide three benchmarks for D’Kitty, their quadruped robot — stand, orient, and walk — that allow rigorous comparison between learning algorithms. D’Kitty offers a low barrier to entry with a cost of $4200, and an assembly time of less than 6 hours. The primary disadvantage of the ROBEL hardware platform is that the servo motors are too slow for agile locomotion and not transparent enough for torque control. Specifically, the servos have a maximum speed of 8 rad/s [7] which is too slow for dynamic locomotion. As a reference, the Pupper actuators reach speeds up to 16 rad/s during 0.7 m/s trotting. In terms of transparency, the D’Kitty servos have a current-control mode similar to the actuators on Pupper, but D’Kitty servos have high-reduction multi-stage gear trains with high nonlinear friction which impedes accurate torque control [19].

A handful of open source quadruped robots have attempted to reduce the gap between inexpensive and open source quadrupeds and high performance systems. Stanford Doggo [12], Solo [8], and MIT Mini Cheetah [11] use low-cost brushless drone motors and open source motor controllers to achieve record agility in certain metrics. However, all three robots required custom-machined parts and laborious assembly that ultimately make them difficult to reproduce at the scale necessary for a benchmarking platform. Of the three, Solo reduces the build complexity the most by relying more heavily on 3D printed parts. However, Solo still uses some custom-machined parts, and most importantly, uses custom actuators, transmissions, and motor controllers, which means it remains unsuitable as an accessible benchmarking platform. As we discuss below, Pupper achieves greater agility than Solo or Doggo while increasing reproducibility.

## III. ROBOT OVERVIEW
Pupper is a 2.1kg, twelve degree of freedom quadruped robot capable of dynamic locomotion. We include a summary of the robot’s specifications, compared against similar quadruped robots, in Tab. I. The following subsections detail the actuator, structure, electronics, and software design of Pupper.

## A. ACTUATOR

The core of Stanford Pupper is its inexpensive, yet high-performance off-the-shelf actuator consisting of the M2006 brushless gearmotor [6] and the C610 motor controller [5].

**1. Brushless gearmotor:** The M2006 brushless gearmotor is shown in Fig. 2. The gearmotor uses a 22mm diameter brushless DC motor and integrated absolute encoder at the input followed by a 36:1 two-stage planetary reduction. The gearbox contains two stacked planetary reductions with a total ratio of 36:1. As summarized in Table II, the overall weight of the actuator is 90g and the peak torque is 1.8Nm.

**2. Motor controller:** The C610 motor controller uses field-oriented-control (FOC) to operate the M2006 in current-control mode. The controller supports current commands to the motor at 1kHz over a CAN connection, and reports back position, velocity, and measured current over the same CAN interface at the same rate.

## B. STRUCTURE

The robot’s frame is constructed out of 3D printed bulk heads and printed circuits boards (PCB) that double as structure and power distribution. The robot uses four identical legs, each with three actuated degrees of freedom. The feet of the robot are made of off-the-shelf wear-resistant rubber bumpers that are easily interchanged depending on the walking surface.

## C. ELECTRONICS

A Teensy 4.0 microprocessor [17] and a Raspberry Pi embedded computer [14] split responsibilities for controlling the robot. The microprocessor receives motor position, velocity, and current estimates and sends motor current commands over CAN at 1kHz. It also reads data from an inertial measurement unit (IMU) and performs onboard state estimation. The embedded computer communicates with the microprocessor over USB serial at 12mbps and is responsible for high-level motion planning, whether it be a reinforcement learning policy, model predictive control, or any other method.

## D. SOFTWARE & CONTROL

The software architecture splits the control responsibilities between the microprocessor and embedded computer. The microprocessor performs low-level actuator control, state estimation, and data logging at 1kHz. Three low-level control methods are supported. In mode 1), the microprocessor passes along motor torque commands sent by the embedded computer. In mode 2), the embedded computer sends joint angle commands and the microprocessor performs joint-space PD control to track the desired positions. This mode is intended to be used with learning-based methods, where the action space is commonly joint position commands and impedance gains. In mode 3), the embedded computer sends desired foot locations in body-relative Cartesian space and the microprocessor performs the task-space impedance control law:

>       Fi = Kp(rref,i − ri) + Kd(vref,i − vi) + Fff,i    (1)
>       τi = JTi Fi                                       (2)

where i is the leg index (1 to 4), Fi is the desired foot force, Kp and Kd are impedance gains, rref,i is the reference foot position, ri is the measured (via forward kinematics) foot position, vref,i is the reference foot velocity, vi is the measured (via forward kinematics) foot velocity, Fff,i is the feed forward force, and JTi is the foot Jacobian.

## E. ROBOT CHARACTERIZATION

State-of-the-art learning and optimization-based methods rely on accurate physical robot models to calculate control commands. The Pupper actuator was tested in a dynamometer to understand the actuator limits, torque-current relationship, and control bandwidth. Actuator torque was measured over varying motor speeds and motor currents to determine a best-fit surface. Fig. 4 summarizes the results. The actuator’s peak torque while doing positive work was 1.8Nm, while the peak torque doing negative work was 3.2Nm. The maximum continuous torque was 1.0Nm. 

The motor friction was modelled well by Coulomb and damping terms, with a R2 value over 0.999, as

>       τf = −0.021Nm sgn(ω) − 0.0045Nmsrad w − 10.0sgn(ω)|τm|    (3)

where τf is the total friction in Nm, ω is the output velocity in rads, and τm is the motor torque at the gearbox input in Nm. The motor torque is given by

>       τm = 0.0069NmAi                                           (4)

where i is the motor current in A. The output torque is modeled as

>       τoutput = 36τm + τf                                       (5)

where τoutput is the output torque, and the factor of 36 comes from the 36:1 reduction ratio.

The friction model was inverted and used to predict motor current necessary to achieve desired torque commands. At constant motor velocities, arbitrary torques can be commanded within 1% error. However, because the inverted model only corrects for velocity-dependent friction, stiction cannot be predicted, which leads to up to 28% torque error when the actuator has zero velocity.

Using the calibrated torque-current relationship, we estimated the rotor inertia by measuring output acceleration at fixed torques. The bandwidth of the actuator was determined by commanding a sinusoidal current and measuring the magnitude of the output torque as a function of frequency. A bode plot of the response is shown in Fig. 5.

## F. ROBOT SIMULATOR
We offer a physically accurate Unified Robot Description Format (URDF) model for rapid experimentation and straight forward integration with robot learning environments such as OpenAI Gym [3]. Work with undergraduates and high school students has highlighted a desire to integrate vision based navigation, motivating the inclusion of photo-realistic textures. The model and simulator environment can be found on the project page [16].

## IV. DESIGN DISCUSSION

## A. TRADEOFFS

We optimized Pupper for easy reproducibility and experimentation while keeping performance sufficient for state-of-the-art control methods. One of the key factors that enabled easy reproducibility was reducing the robot size. While large quadruped robots have higher payload capacities and can step over larger obstacles, these advantages are not necessary to benchmark different controllers’ core agility and robustness. Instead, by making Pupper small and light, we were able to use a 3D printed frame instead of custom-machine aluminum pieces. The light weight also avoids the need for costly, larger actuators. Above all, Pupper’s small form factor and low weight make it easy to use in a remote-work setting since it does not need a support crane or other apparatuses for testing.

We were further able to increase accessibility while maintaining performance by using hobbyist brushless motors instead of using more expensive industrial motors. Hobbyist brushless motors with high specific torque have been used in many of the state-of-the-art quadruped robots like Mini Cheetah and Solo, but not on robots as small as Pupper. However, decreasing the motor size decreases the motor torque hyperlinearly, so larger reduction ratios are needed at small scales to maintain adequate torque. The disadvantage of higher reductions ratios are greater inertia and greater inefficiency, both qualities that hurt actuator transparency [15]. While a higher quality actuator such as the Maxon ECXTQ22L BL KL A STD 24V motor and GPX22HP 35:1 gearbox would increase actuator efficiency from 72% to 93% and reduce inertia, the actuator cost would increase by more than ten times [13]. In this regard Pupper sacrifices torque-controllability and transparency for greater accessibility.

An additional tradeoff made was to co-locate the knee motor at the knee joint. Unlike the common pattern of mounting the knee motor at the hip such as in Solo and MIT Mini Cheetah–which reduces leg inertia–Pupper mounts the knee motor at the knee to eliminate the need for an additional transmission. We found that the added leg inertia did not preclude agile trotting and other locomotion.

The robot structure was simplified wherever possible for better accessibility. The 3D printed structure eliminates the need for custom-machined parts, which the authors found to be critical for helping undergraduate students take part in the work. 3D printing the majority of robots part enables experimenters to easily modify the robot geometry and material choice.

## B. OPEN SOURCE

The design for Pupper is entirely open source under the MIT License and all documentation are available on the project page, https://stanfordstudentrobotics.org/quadruped-benchmark. We include instructions for sourcing parts and a bill of materials, which totals less than 2000 USD for the entire robot including fabrication costs. The project page also hosts exhaustive documentation for completing the hardware assembly and software bring-up.

## V. BENCHMARK TASKS

We propose a set of tasks designed for real-world benchmarking so that different controllers can be compared on equal footing across different robots and institutions. The benchmarks are designed to be repeatable and simple enough that the test can be quickly attempted for fast iteration and data collection.

## A. SPRINT

**1. Task overview:** Illustrated in Fig. 7a, Sprint requires Pupper to traverse an unobstructed five-meter course as fast as possible. While there are no constraints placed on the robot heading or the path it takes, Pupper must begin with zero velocity. The benchmark score is the average speed over the course, taken as the time to traverse the course divided by the five meter length. While not factored into the score, metrics including motor odometry and IMU data are logged by the onboard microcontroller to allow offline comparison of metrics including cost of transport and orientation error. Additional details are included on the project page [16].

**2. Task rationale:** Achieving fast locomotion has long been a goal of the legged robotics research community, but has often not been tested under standardized conditions. High speed locomotion has garnered much research because it requires fundamental understanding of locomotion to tightly coordinate full-body motion while managing ground impacts and destabilizing perturbations. However, new control techniques are often introduced in conjunction with novel mechanical designs, which makes it difficult to disentangle the separate contributions of control and mechanical design. Sprint is designed to provide a reproducible way to measure maximum forward speed. We hope to encourage researchers and students to compare both learned and hand designed gaits on a common platform, and to make the implementations of those gaits accessible to others.

## B. SCRAMBLE

**1. Task overview:** This task requires Pupper to traverse a set of obstacles arranged in predefined locations shown in Fig. 7b. As with the Sprint task, Pupper must start with zero velocity and the goal is to traverse the course as quickly as possible. We expect to see methods based on proprioception alone, but also methods based on vision and terrain estimation. The benchmark score is time taken to complete the course. We expect that the offline metrics may be more interesting and in some cases even more insightful than just the benchmark score. Additional details are included on the project page [16].

**2. Task rationale:** This benchmark, consisting of a set of relatively tall obstacles, was designed to challenge many of the common locomotion models. In particular, the model predictive control method used in many recent quadrupeds assumes that the terrain is flat [2], [4]. We hope this benchmark showcases generalist controllers that rely on fewer assumptions about the terrain. Like with Sprint, we hope to see students and researchers publish their solutions for easy reproduction.

## C. REFERENCE CONTROLLER

We implemented a trotting controller to serve as a baseline for the two benchmark tasks. This controller runs on the embedded computer and generates foot position targets as a function of time and desired velocity in the horizontal plane. The architecture is similar to the position-based controller in Stanford Doggo [12] and the Foot Trajectory Generator (FTG) architecture formalized in [10]. Various parameters can be tuned to optimize for different gaits and terrain, including stepping height, trotting frequency, and stance/swing gait percentages. The onboard microcontroller uses task-space impedance control to calculate motor torques needed to track the foot position targets.

## VI. EXPERIMENTS

We tested the hardware platform and reference controller over a variety of terrain and demonstrated reliability, robustness and agility. Fig. 8 and the supplementary video highlight the robot trotting omnidirectionally over gravel, cement, and loose natural terrain. The controller is robust to loose terrain like tanbark and pebbles, and can recover from unexpected drops going over curbs. On flat ground, the reference controller achieved a stable forward and backwards speed of 0.8 m/s, a sideways speed of 0.4 m/s, and a maximum turning rate of 2.5 rad/s. The forward speed of 0.8 m/s is comparable to the 0.8m/s trotting speed achieved with Anymal, but less than that of the more agile MIT Cheetah 3, which achieves 3.0m/s using its convex model-predictive controller.

The benchmarks tasks were tested across three different Pupper robots, each built at different universities, in order to demonstrate replicability and to establish baseline scores. One of the robots was built by the authors, and the two others were built by undergraduate engineering students at Massachusetts Institute of Technology and Worcester Polytechnic Institute in under a day. The robots completed the Sprint task with an average score of 0.66, and a standard deviation of 0.025. Fig. 9, illustrates the mean and interquartile ranges of the benchmark scores across trials for each of the robots. Fig. 10 compares the deviation and overall consistency of electrical power used by the three robots over a single trial on the Sprint benchmark. The Scramble task was conducted with one robot over several trials, resulting in an average score of 34.6 with a standard deviation of 4.3.

## VII. EDUCATIONAL OUTREACH

One of the primary impacts we hope to have with this project is to introduce robotics research opportunities at the high school and college level. We have initiated several collaborations with high school and college educators to design curriculum and bring Pupper into their classrooms. Through partnership with HandsOnRobotics, we plan to donate robots at the community and high school level to kick start a competitive robotics league centered around the Pupper platform.

## VIII. CONCLUSION

We introduced an accessible, open source, and high performance quadruped robot to make legged robotics research more accessible and reproducible. The robot is small and lightweight which makes it ideal for when lab environments are not available such as in high school, undergraduate, and remote-work environments. Two benchmarks were introduced to provide a standard measure from which to compare progress on controller research. Through our ongoing collaborations with educators, and by the release of extensive documentation and open source design, we hope to push forward the state-of-the-art while engaging more students in legged robotics research.

## ACKNOWLEDGMENT

We thank the members of Stanford Student Robotics for their ongoing support of this project including Tarun Punnoose, Ian Chang, and Parthiv Krishna; Jeremy Trilling and Gregory Xie for replicating Pupper and contributing feedback; Mark Bowers for simulator integration; and Professor Zachary Manchester for discussion along the way.

## REFERENCES

[1] M. Ahn, H. Zhu, K. Hartikainen, H. Ponte, A. Gupta, S. Levine, and V. Kumar (2019) ROBEL: RObotics BEnchmarks for Learning with low-cost robots. In Conference on Robot Learning (CoRL), Cited by: §II.

[2] G. Bledt, M. J. Powell, B. Katz, J. Di Carlo, P. M. Wensing, and S. Kim (2018) MIT cheetah 3: design and control of a robust, dynamic quadruped robot. In 2018 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), pp. 2245–2252. Cited by: §I, §V-B2.

[3] G. Brockman, V. Cheung, L. Pettersson, J. Schneider, J. Schulman, J. Tang, and W. Zaremba (2016) OpenAI gym. External Links: 1606.01540 Cited by: §III-F.

[4] J. Di Carlo, P. M. Wensing, B. Katz, G. Bledt, and S. Kim (2018) Dynamic locomotion in the mit cheetah 3 through convex model-predictive control. In 2018 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), pp. 1–9. Cited by: §I, §V-B2.

[5] DJI c610 brushless motor controller. Note: https://store.dji.com/product/rm-c610-brushless-dc-motor-speed-controlAccessed: 2020-10-30 Cited by: §III-A.

[6] DJI m2006 brushless motor. Note: https://store.dji.com/product/rm-m2006-p36-brushless-motorAccessed: 2020-10-30 Cited by: §III-A.

[7] Dynamixel xm430-w210-r. Note: http://www.robotis.us/dynamixel-xm430-w210-r/Accessed: 2020-10-30 Cited by: §II.

[8] F. Grimminger, A. Meduri, M. Khadiv, J. Viereck, M. Wüthrich, M. Naveau, V. Berenz, S. Heim, F. Widmaier, T. Flayols, et al. (2020) An open torque-controlled modular robot architecture for legged locomotion research. IEEE Robotics and Automation Letters 5 (2), pp. 3650–3657. Cited by: §II.

[9] M. Hutter, C. Gehring, D. Jud, A. Lauber, C. D. Bellicoso, V. Tsounis, J. Hwangbo, K. Bodie, P. Fankhauser, M. Bloesch, et al. (2016) Anymal-a highly mobile and dynamic quadrupedal robot. In 2016 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), pp. 38–44. Cited by: §I.

[10] A. Iscen, K. Caluwaerts, J. Tan, T. Zhang, E. Coumans, V. Sindhwani, and V. Vanhoucke (2019) Policies modulating trajectory generators. External Links: 1910.02812 Cited by: §V-C.

[11] B. Katz, J. Di Carlo, and S. Kim (2019) Mini cheetah: a platform for pushing the limits of dynamic quadruped control. In 2019 International Conference on Robotics and Automation (ICRA), pp. 6295–6301. Cited by: §II, §II.

[12] N. Kau, A. Schultz, N. Ferrante, and P. Slade (2019) Stanford doggo: an open-source, quasi-direct-drive quadruped. In 2019 International Conference on Robotics and Automation (ICRA), pp. 6309–6315. Cited by: §II, §V-C.

[13] Maxon motor. Note: https://www.maxongroup.com/maxon/view/content/indexAccessed: 2020-10-30 Cited by: §IV-A.

[14] Raspberry pi. Note: https://www.raspberrypi.org/Accessed: 2020-10-30 Cited by: §III-C.

[15] S. Seok, A. Wang, Meng Yee Chuah, D. Otten, J. Lang, and S. Kim (2013) Design principles for highly efficient quadrupeds and implementation on the mit cheetah robot. In 2013 IEEE International Conference on Robotics and Automation, Vol. , pp. 3307–3312. External Links: Document Cited by: §IV-A.

[16] Stanford pupper benchmark. Note: https://stanfordstudentrobotics.org/quadruped-benchmarkAccessed: 2020-10-30 Cited by: §III-F, §V-A1, §V-B1.

[17] Teensy 4.0. Note: https://www.pjrc.com/store/teensy40.htmlAccessed: 2020-10-30 Cited by: §III-C.

[18] E. Todorov, T. Erez, and Y. Tassa (2012) MuJoCo: a physics engine for model-based control. In 2012 IEEE/RSJ International Conference on Intelligent Robots and Systems, Vol. , pp. 5026–5033. External Links: Document Cited by: Fig. 6.

[19] A. Wang and S. Kim (2015-06) Directional efficiency in geared transmissions: characterization of backdrivability towards improved proprioceptive control. Proceedings - IEEE International Conference on Robotics and Automation 2015, pp. 1055–1062. External Links: Document Cited by: §II.
