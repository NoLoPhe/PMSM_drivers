Link: https://ieeexplore.ieee.org/abstract/document/8793865

# Mini Cheetah: A Platform for Pushing the Limits of Dynamic Quadruped Control

Benjamin Katz[1], Jared Di Carlo[2] and Sangbae Kim[1]

Authors are with the the 1Department of Mechanical Engineering at the Massachusetts Institute of Technology, and the 2Department of Electrical Engineering and Computer Science at the Massachusetts Institute of Technology, Cambridge, MA, 02139, USA. Corresponding Author: benkatz@mit.edu

**Abstract** — Mini Cheetah is a small and inexpensive, yet powerful and mechanically robust quadruped robot, intended to enable rapid development of control systems for legged robots. The robot uses custom backdriveable modular actuators, which enable high-bandwidth force control, high force density, and robustness to impacts. Standing around 0.3 m tall and weighing 9 kg, Mini Cheetah can easily be handled by a single operator. We have demonstrated dynamic trot, trot-run, bounding, and pronking gaits on the robot to speeds of up to 2.45 meters per second using Convex Model-Predictive Control (cMPC). In addition to locomotion, we have used the robot to execute 360◦ backflips, with trajectories generated using offline nonlinear optimization.

## I. INTRODUCTION

Mini Cheetah is a lightweight, inexpensive, and high performance quadruped robot. Although the robot has a variety of uses in its own right, our primary motivation in developing the Mini Cheetah is to enable rapid development of control systems for legged robots, by allowing fearless experimentation on hardware.

Several other small qudruped robots have been developed with similar goals of low-cost and easy, safe experimentation. Both the predecessor to this robot, the Super Mini Cheetah [1], and Minitaur [2] feature highly backdriveable actuators, making them mechanically robust and allowing control of ground reaction forces through proprioception, but have only two degrees of freedom per leg. Additionally, the symmetric 5-bar mechanism used in their legs occupies a large volume and limits range of motion. Cheetah Cub from EPFL [3] uses two position-controlled servomotors per leg, and springs to store and release energy during the gait cycle. The Little Dog robot from Boston Dynamics has 3 degrees of freedom per leg and a large range of motion, but is relatively slow and position-controlled, not capable of controlling ground reaction forces [4].

At 9 kg, Mini Cheetah can be easily handled by a single operator. It features modular actuation designed for highbandwidth torque control, torque density, and robustness to high-speed impacts. Despite its small size and low cost, Mini Cheetah has demonstrated highly dynamic behaviors including trot, trot-run, bounding, and pronking gaits at speeds up to 2.45 m s−1, and 360◦ backflips from standing.

We believe the significantly improved performance and robustness of Mini Cheetah compared to its predecessors make the robot ideal for extremely dynamic experiments, and pushing the limits of quadruped robot performance.

## II. DESIGN

The mechanical design and actuation approach taken with the Mini Cheetah largely overlaps with its larger relative, the MIT Cheetah 3 [5]. It features a large range of motion on all degrees of freedom, low-inertia limbs, and backdriveable actuators, with no need for torque or force sensors or series compliance. Mini Cheetah is roughly 60% the length of Cheetah 3, with 0.21 meter and 0.19 meter link-lengths for the uppper and lower links respectively, and 0.38 meter between the front and back legs. While Cheetah 3 has highly integrated actuation with custom electric motors and transmissions, Mini Cheetah has been designed for modularity and low cost.

### A. Modular Actuators

Mini Cheetah uses identical modular actuators on every degree of freedom. A similar approach was used in the quadruped ANYmal, with the series-elastic ANYdrive actuator [6]. Each actuator contains an electric motor, singlestage planetary transmission, and power electronics. Using identical, self-contained actuators at each degree of freedom simplifies robot design, allows for easy repairs and modification of the robot, and lowers cost because of the number of identical parts. The design and performance of these actuators is discussed in detail in Section III.

### B. Mechanical Design

The robot’s four identical legs were designed to maximize range of motion while minimizing limb mass and inertia. The actuators have sufficient internal bearings for the three degrees of freedom to be serially attached with no additional support structure, which would add weight and limit range of motion. The hip and knee motors are located co-axially at the hip, to minimize the moment of inertia. Torque is transmitted to the knee joint through a Gates Poly Chain belt transmission which provides an additional 1.55:1 gear-up.
Similar to the roller chain knee transmission in Cheetah 3, the belt allows ± 155◦ range of motion from fully extended, so the robot can operate in both knee-forward and kneebackward configurations. The belt transmission does reduce torque control bandwidth at the knee by introducing a 30 Hz belt-actuator resonance. So far, we have not observed the belt compliance to affect locomotion performance, but if necessary the belt width can be easily increased to improve stiffness, at little weight or size penalty to the robot.

The abduction/adduction (ab/ad) joint can rotate ± 120◦, the hip joint ± 270◦ (limited by wire length), and the
knee joint ± 155◦; see Figure 3. This range of motion will allow the robot to operate identically forwards, backwards, or upside-down, roll its body by 90◦ to fit through narrow gaps, and climb obstacles much taller than its leg length.

In a full-collapsed configuration, near worst-case for vertical force production, each leg is capable of producing 150 N (1.7 bodyweights) peak and over 60 N (0.7 bodyweights) continuous vertical force. This indicates the robot should have an additional payload capability of around its own bodyweight.

In addition to improving proprioceptive force control, low limb inertia and low reflected actuator inertia make the robot capable of extremely fast leg-swings. The legs have an angular acceleration capability of 1700 rad s−2
at the hip and ab/ad joints, and 5000 rad s−2 at the knee joint. This corresponds to a linear acceleration of 875 m s−2, or 89 G’s, at the foot using only the knee motor.

The body of the robot is a lightweight sheet aluminum monocoque which houses the battery, logic power supply, VN-100 IMU, wireless receiver and control computer.

### C. Electronics and Computing

Locomotion and other high-level control tasks are executed on an UP Board low-power single board computer with a quad-core Intel Atom CPU and 4 GB RAM, running Linux with the CONFIG PREEMPT RT patch for sudorealtime operation. High level communication and data logging is accomplished using Lightweight Communication and Marshaling (LCM) [7]. The robot has sufficient internal space and power supply capability to add more computing as needed, for processing vision information for example, and LCM will allow easy integration of additional sensor data. The computer communicates with the 12 actuators through a custom quad CAN bus interface. The control, state estimation, and actuator communication loops run at 1 kHz, although our locomotion control typically does not run every loop iteration. Control code and simulation environment are largely shared with the Cheetah 3 Robot, which allows for easy transfer of control strategies between the two machines.

The robot is powered by an off-the-shelf 24 V, 120W h Li-Ion battery designed for cordless power tools, which is
rugged, has built-in battery management, easy charging, and a simple mechanical interface for quick battery-swapping. Logic and computer are powered by the same battery, with an isolated DC-DC converter to step down the battery voltage and provide electrical isolation between the power electronics and computer.

## III. MODULAR ACTUATORS

Mini Cheetah is actuated by 12 identical low-cost, modular actuators. These were designed based on the actuation paradigm of the MIT Cheetah robots, using a high torque density electric motor coupled to a low-ratio transmission to achieve high torque density, backdriveablility, and high bandwidth force control through proprioception [8]. In small quantities, the hardware cost of the actuators is around $300, making the total cost of actuation hardware for the 12 degree of freedom Mini Cheetah around $3600.

In addition to the Mini Cheetah, these actuators have been used in a 6-DOF lower-body biped robot used for wholebody bilateral teleoperation [9].

### A. Design

While the MIT Cheetah uses custom-designed electric motors optimized for torque density, this actuator uses motors originally designed for remote control drones and airplanes, which are manufactured in huge quantities, at very low cost. These motors have been tightly integrated with a 6:1 single-stage planetary gear reduction, motor controller with built-in position sensor and joint-level control capabilities, mechanical interface which can handle substantial moment loads for directly attaching limbs to the actuators, and daisy-chainable power and communication to simplify wiring.

The electric motors used are the iFlight ex-8, which have geometry identical to the T-Motor U8 used in Minitaur[2], but are significantly lower cost (in our case $66 without the stator housing or bearings). This particular motor was chosen for its large airgap radius of (40.5 mm), a stack length of 8.2 mm, and a large number of pole-pairs, which give it particularly high torque density for an off-the-shelf motor.

To minimize the axial length of the actuator, the planetary gear set is placed inside the center of the stator. The hardened pins which support the planet needle roller bearings extend through the output of the actuator, and serve as locating and torque transmission features to the output. The planetary transmission uses all stock gears made by Misumi, and as a result has roughly 0.3◦ of backlash at the output. Whereas many robotics applications require high position accuracy, and therefore low backlash transmission, through our work with the MIT Cheetah robots we have not observed backlash to be of particular importance for legged locomotion.

A custom motor controller with integrated magnetic encoder IC is located behind the rotor. The motor controller is designed for 24V nominal operation, 30A continuous and 40A peak current. The controller handles field oriented control of motor currents at a loop rate of 40 kHz and closedloop bandwidth of up to 4.5 kHz, as well as position and velocity control if desired. The controller receives torque, position, velocity and gain commands, and returns position, velocity, and estimated torque over CAN bus at a rate of up to 4kHz/ # of actuators. For robots with many degrees of freedom, like the Mini Cheetah, multiple CAN networks can be used to keep communication rates high. In the case of the Mini Cheetah, one network is used per-leg, allowing communication between all joints and the control computer to happen at 1 kHz.

### B. Design for Impact

Robustness to impacts is a critical feature in actuators for legged robots. Contacts with the world at non-zero velocity is inevitable, even if the robot’s control strategies work hard to avoid them. Any uncertainty in the terrain, any error in robot state estimation, and any disturbance can cause impacts with significant velocity, and legged machines must be mechanically robust to this.

For electromagnetically actuated robots, typically the transmission between the electric motor and the limbs of
the robot are the limiting mechanical elements. Series Elastic Actuators (SEA) intentionally place a spring on the output of the actuator to mitigate impact loads on their transmissions (as well as enabling closed-loop torque control) [10]. In other robots, like the MIT Cheetah, the inherent compliance in their transmissions, links, and feet is sufficient to limit the impact forces.

The open-loop output torque bandwidth of an actuator with the output locked is roughly equal to the natural frequency of the actuator inertia and compliance [11]. While closed-loop torque control (such as with a Series Elastic Actuator) allows for higher small-signal torque control bandwidth than the natural frequency, the additional bandwidth comes at the cost of additional motor torque when operating above the natural frequency, as the motor has compensate for the natural low-pass dynamics of the system [12]. Furthermore, for large output torques, as saturation of the closed-loop control comes into effect, torque bandwidth drops significantly below the small-signal bandwidth [6]. So the actuator inertia-compliance natural frequency is a reasonable proxy for torque control bandwidth. Based on actuator inertia and compliance, we can estimate how strong transmission components must be to achieve a given torque control bandwidth and survive an impact at a given speed, or, conversely, design the rest of the robot such that it does not exceed the impact tolerance of the actuator.

Two possible scenarios are as follows: Compliance between the output of the transmission and the robot link, like an SEA; and compliance between the robot link and the world, as illustrated in Figure 6. In both cases, the rotor has inertia J1, the compliance has a stiffness K, the output link has an inertia J2, and there is a transmission of ratio N between the input and output of the transmission. The output link collides with the world at angular velocity ω, with the point of contact coming to an instantaneous stop.

Peak forces are seen in the transmission when the kinetic energy stored in the rotor (and of the link, in Case 2) has been completely transferred to potential energy stored in the compliance. At this point, the compliant element will deflect to angle K, and peak force at the input of the transmission for Case 1, 2 respectively will be:

The natural frequencies of the two systems are:

If the spring constants for the two cases are chosen such that the natural frequencies with the output locked are identical, we find that, in both cases, the peak torques seen at the input to the transmission are also identical, and equal to:

Peak torque after impact is proportional to collision speed, gear ratio, rotor inertia, and open-loop natural frequency (essentially torque control bandwidth).

By inserting the maximum allowable torque for the limiting component of the actuator into (5) (in this case, the sun gear in the planetary gearbox), and choosing a maximum allowable collision speed, we can determine how stiff the leg and foot of the robot must be to satisfy the maximum allowable torque, and what open-loop bandwidth is possible. For our actuator, the allowable torque on the sun gear is 11 N m. With a gear ratio of N = 6, rotor inertia of J1 = 0.000 072 kg m2, and a maximum actuator velocity of around 40 rad s−1, the maximum allowable natural frequency for a full-speed collision is 101 Hz. Even higher bandwidth can be achieved if the maximum collision velocity is limited: in all likelihood, a robot using these actuators will not experience a maximum-speed collision with a rigid object. These numbers give confidence that the actuator will handle impacts well without any additional compliance built into the actuator, as the stiffness of the links, other transmission elements, and foot of the robot will likely bring the natural frequency below 101 Hz.

Worth noting, the peak torque tolerable by the transmission is significantly greater than the peak torque which the motor can produce - in this case, roughly a factor of 4. This means that for high-velocity impacts, using the motor to actively “run away” from impacts to reduce the load on the transmission would have little effect, only reducing impact torque by around 1/4 best-case. If the transmission were sized only to match the electromagnetic torque capabilities of the motor, then the achievable bandwidth for a given maximum impact velocity would be significantly reduced.

### C. Performance

Actuator specifications are summarized in Table I

The efficiency of the planetary gearbox was measured with a torque sensor on the output of the actuator. Gear and bearing friction was observed be dependent on torque and rotation direction, with negligible speed dependence within the operating range of the actuator. The gear and bearing friction is well modeled by the following expression:

Where ω is is the angular velocity of the of output, τmotor = τrotor · gear ratio, and output torque is τoutput = τmotor+τfriction. This corresponds to a transmission efficiency of greater than 90% at torques above 1.5 N m, and greater than 94% at 4.5 N m and above.

## IV. EXPERIMENTS

Becuase the actuation and design similarities between the Mini Cheetah and Cheetah 3 robots, we have been able to use the same state estimation and control strategies used on Cheetah 3 on the Mini Cheetah by simply changing the physical robot parameters used for control and tuning various weights and gains. So far, we have implemented the quadratic program (QP) based balance control discussed in [5], as well as the convex Model Predictive Controller (cMPC) for locomotion described in [13]. 

To demonstrate the dynamic capabilities of the robot, and the utility of having a small, robust platform on which to experiment, we used offline nonlinear optimization to generate a backflipping trajectory, which we were able to successfully execute on the robot. To our knowledge, this is the first example of 360◦ flip on a quadruped robot. The supplemental video [14] includes highlights from balance, locomotion, and backflip testing.

### A. Locomotion

For locomotion, we use Convex Model Predictive Control (cMPC) as describe in [13]. The cMPC determines ground reaction forces for the stance feet by solving an MPC problem using a highly simplified model of the robot dynamics, and a prediction horizon spanning one gait cycle. The MPC can be formulated as a QP, which can be solved in around 5 ms on the onboard computer. Ground reaction forces commanded by the MPC are directly used to calculate motor torques by

where J ∈ R 3×3 is the foot Jacobian, and f is the vector of MPC forces in the robot body frame. No filtering or further smoothing is applied to the resulting torques. Although this simplification neglects all leg dynamics, it has proved sufficient for this machine as well the other MIT Cheetah robots.

For experiments so far, the gait timing has been fixed, and foot placement locations are calculated based on a heuristic similar to [15], but with additional angular velocity, capture point, and high-speed turning terms. With cMPC, the robot has achieved trotting at up to 2.45 m s−1 forwards and backwards, strafing sideways at 1.0 m s−1, yawing at a rate of 4.6 rad s−1, and linear accelerations of up to 5 m s−2. This makes the Mini Cheetah the fastest among sub-10 kg quadruped robots, and comparable in speed to most largescale quadruped robots. At these speeds we are far from the maximum actuator speeds and torques, so we expect with further controller development, Mini Cheetah will be able to move significantly faster.

### B. Backflips

The first controller implemented on the Mini Cheetah was a back-flipping controller. To our knowledge, this is the first example of a quadruped robot doing a 360◦ flip, although the 2D and 3D Biped robots from the MIT Leg Lab and the biped Atlas from Boston Dynamics have done a somersault [16] and a backflip off a raised platform [17], respectively. While not inherently useful, flipping served as an excellent stress-test of the robot platform, involving high peak torques and powers and high impact speeds upon landing.

**1. Backflip Trajectory Optimization:** To generate a backflip trajectory, we used the open-source CasADi library [18] to set up a trajectory optimization problem to be solved by IPOPT [19]. The optimization was done on a 5-link sagittal plane model of the robot, which included five rigid links, as well as the effects of the rotors of the actuators. The 2D model was significantly faster than our full 3-D optimization, and captured the relevant dynamics for a backflip. We found it was not necessary to specify a nonzero cost function; simply allowing the optimization to find a feasible backflip given our constraints was sufficient. Some of the constraints in addition to the dynamics used in the optimization were:

  - Initial and final position and velocity: The robot starts in a configuration close to the ground, so the full workspace of the legs can be used, and with zero velocity. The landing configuration has the legs more extended, so they can absorb the impact. The ending orientation of the body was set to around −1.9π radians.
  - Fixed contact sequence between the feet and the ground: i.e. the takeoff timing of the front and back pairs of legs was fixed.
  - All points on the robot should remain above the ground.
  - The front and back legs should not intersect.
  - Ground reaction forces must be in a friction cone: |fx| ≤ µfz.
  - Absolute and speed-dependent actuator torque limits from current and voltage limits.

The problem was set up as direct transcription and included 80 time steps, with a .01 integration time step, which resulted in around optimization 2000 variables (the states and control inputs at each time step) and 4000 constraints. The optimization can be easily set up to generate many different dynamic behaviors. With different sets of constraints, we have been able to (in simulation) jump while yawing in 3D, jump up onto and down from a platform, and backflip over a 30 cm wall.

**2. Backflip Control:** Whereas the 3D biped was able to adjust its angular velocity mid-air by changing its moment of inertia, in order to land in the correct orientation, such a controller was not possible with the Mini Cheetah: The robot’s moment of inertia about its pitch axis negligibly changes as the legs extend and retract, so it is not possible for the robot to significantly change its angular velocity during flight. Sticking the landing instead required accurate takeoff height and angular velocity. As the robot pitches at a rate of over 500 ◦s−1 while flipping, very slight errors in timing make landing impossible. Furthermore, at the time of execution, the robot’s IMU had not yet been integrated, so no feedback on body orientation was possible during the takeoff period.

The flips were executed by using the joint torques from the optimization as feed-forward joint torque commands, with PD joint feedback to track the optimized joint trajectories.

After some tuning of the final configuration of the legs for landing, this approach was over 90% successful on the hardware. Failures were typically due to excessive roll during the flight period, which resulted in a poor landing configuration. Rotation about the roll axis is unstable during the flip, so any non-zero roll velocity on takeoff resulted in a unfavorable landing orientation. Without body feedback, the robot was only able to stabilize around 40◦ roll upon landing. Now that the appropriate sensing and state estimation have been integrated in to the robot, we would like to revisit the flipping and landing controllers to regulate the body orientation during takeoff and landing using the cMPC or other control techniques, which should improve repeatability and robustness to varying conditions like uneven or sloped ground.

During the flips, the robot’s COM reached a height of 0.7 m. Joint torques and output power required for the flip are shown in Figure 11. Mechanical output power of the robot reached a peak of 690W, just before the rear legs take off the ground. Velocity of the rear feet just prior to impact reached over 7 m s−1 - equivalent to falling from over 2.5 m.

V. CONCLUSIONS

Mini Cheetah is a new lightweight, low-cost quadruped robot, featuring similar dynamic capability for its size to the MIT Cheetah 3 robot. It employs modular actuators, which were designed to simultaneously deliver high torque density, torque control bandwidth, and tolerance to external impacts. Because of its size, performance, and robustness, it enables rapid experimentation in hardware of highly dynamic behaviors: It has demonstrated dynamic locomotion using cMPC at speeds of up to 2.45 m s−1, turning at 4.6 rad s−1, and over 0.5 G’s of linear acceleration. The robot has also performed backflips generated by offline trajectory optimization, which we believe to be a first for a quadruped robot.

## REFERENCES

[1] W. Bosworth, S. Kim, and N. Hogan, “The mit super mini cheetah: A small, low-cost quadrupedal robot for dynamic locomotion,” pp. 1–8, Oct 2015.
[2] G. Kenneally, A. De, and D. E. Koditschek, “Design principles for a family of direct-drive legged robots,” IEEE Robotics and Automation Letters, vol. 1, pp. 900–907, July 2016.
[3] A. Sprowitz, A. Tuleu, M. Vespignani, M. Ajallooeian, E. Badri, and A. J. Ijspeert, “Towards dynamic trot gait locomotion: Design, control, and experiments with cheetah-cub, a compliant quadruped robot,” The International Journal of Robotics Research, vol. 32, no. 8, pp. 932–950, 2013.
[4] M. P. Murphy, A. Saunders, C. Moreira, A. A. Rizzi, and M. Raibert, “The littledog robot,” The International Journal of Robotics Research, vol. 30, no. 2, pp. 145–149, 2011.
[5] G. Bledt, M. J. Powell, B. Katz, J. D. Carlo, P. M. Wensing, and S. Kim, “Mit cheetah 3: Design and control of a robust, dynamic quadruped robot,” 2018 IEEE International Conference on Intelligent and Robots (IROS), 2018.
[6] M. Hutter, C. Gehring, D. Jud, A. Lauber, C. D. Bellicoso, V. Tsounis, J. Hwangbo, K. Bodie, P. Fankhauser, M. Bloesch, R. Diethelm, S. Bachmann, A. Melzer, and M. Hoepflinger, “ANYmal - a highly mobile and dynamic quadrupedal robot,” in IEEE/RSJ International Conference on Intelligent Robots and Systems, pp. 38–44, Oct 2016.
[7] A. S. Huang, E. Olson, and D. C. Moore, “Lcm: Lightweight communications and marshalling,” in IEEE/RSJ International Conference on Intelligent Robots and Systems, pp. 4057–4062, Oct 2010.
[8] S. Seok, A. Wang, M. Chuah, D. J. Hyun, J. Lee, D. Otten, J. Lang, and S. Kim, “Design principles for energy-efficient legged locomotion and implementation on the mit cheetah robot,” Mechatronics, IEEE/ASME Transactions on, vol. 20, pp. 1117–1129, June 2015.
[9] J. Ramos, B. Katz, M. Y. Chuah, and S. Kim, “Facilitating modelbased control through software-hardware co-design,” 2018 IEEE International Conference on Robotics and Automation (ICRA), 2018.
[10] G. A. Pratt and M. M. Williamson, “Series elastic actuators,” in Proceedings 1995 IEEE/RSJ International Conference on Intelligent Robots and Systems. Human Robot Interaction and Cooperative Robots, vol. 1, pp. 399–406 vol.1, Aug 1995.
[11] P. M. Wensing, A. Wang, S. Seok, D. Otten, J. Lang, and S. Kim, “Proprioceptive actuator design in the MIT Cheetah: Impact mitigation and high-bandwidth physical interaction for dynamic legged robots,” IEEE Transactions on Robotics, vol. 33, no. 3, pp. 509–522, 2017.
[12] N. Paine, J. Mehling, J. Holley, N. A. Radford, G. Johnson, C.-L. Fok, and L. Sentis, “Actuator control for the nasa-jsc valkyrie humanoid robot: A decoupled dynamics approach for torque control of series elastic robots,” J. Field Robotics, vol. 32, pp. 378–396, 2015.
[13] J. DiCarlo, P. M. Wensing, B. Katz, G. Bledt, and S. Kim, “Dynamic locomotion in the MIT Cheetah 3 through convex model predictive control.” IROS 2018.
[14] “MIT Mini Cheetah.” https://youtu.be/xs7UJoPRVr0, 2019. [Online; accessed 25-2-2019].
[15] M. H. Raibert, Legged robots that balance. Cambridge, MA, USA: MIT Press, 1986.
[16] R. Playter and M. Raibert, “Control of a biped somersault in 3D,” in Proc. of the IEEE/RSJ Int. Conf. on Intelligent Robots and Systems, vol. 1, pp. 582–589, July 1992.
[17] “What’s New, Atlas?.” https://youtu.be/fRj34o4hN4I, 2017. [Online; accessed 9-4-2018].
[18] J. A. E. Andersson, J. Gillis, G. Horn, J. B. Rawlings, and M. Diehl, “CasADi – A software framework for nonlinear optimization and optimal control,” Mathematical Programming Computation, In Press, 2018.
[19] A. Wachter and L. T. Biegler, “On the implementation of an interior- point filter line-search algorithm for large-scale nonlinear programming,” Mathematical Programming, vol. 106, pp. 25–57, Mar 2006.
