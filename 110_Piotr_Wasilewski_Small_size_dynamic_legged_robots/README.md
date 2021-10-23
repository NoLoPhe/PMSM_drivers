Link: https://pwwprojects.blogspot.com/2021/10/my-bachelors-thesis-brushless-actuator.html

# SUBJECT: A STAND-ALONE BRUSHLESS ACTUATOR FOR SMALL-SIZE DYNAMIC LEGGED ROBOTS  

AUTHOR: Piotr Wasilewski

SUPERVISOR: Maciej Ciężkowski PhD.

**Abstract** - The aim of this thesis is to design, build and test a small brushless actuator for application in dynamic legged robots. Nowadays, legged robots are increasingly gaining attention. They start to show up not only as prototypes, but also as ready solutions for fulfilling supervisory tasks in factories and places too remote or dangerous for people. Due to the fact that these robots have to maintain a very stable pose and quickly react to disturbances, their actuators have to meet some specific requirements. In this thesis, a brushless actuator for small quadrupedal robots (sub 5kg) is presented. The module is cheap, easy to manufacture, and based on easily accessible components. First, an overview of other legged robot actuators is presented. Then, the basic rules governing electric motors are described, as well as their ability to produce torque. The most optimal algorithm is chosen and implemented in the virtual simulation environment. Next, the mechanical structure and electronic controller is described in detail. The controller main software and bootloader code for updating firmware through CANFD bus is introduced. Finally, the results of real-life tests are presented – the recorded control waveforms, static torque data, and automatic motor parameter identification. In the end, a 3D model of a single 3DoF robot leg based on custom actuators is shown.

**Acknowledgments** - I received some great support while writing this thesis, and I would like to say thank you to: 
 My supervisor, Maciej Ciężkowski PhD, for explaining the control theory aspects related to motor control, and his dedication to the subject.

 Marcin Lis, for multiple thesis reviews, improvement suggestions, and comments.

 Maciej Słowik PhD, for a thorough thesis check, finding any errors and inconsistencies.

 Rafał Grądzki PhD, for the help with getting the necessary measurement equipment.

 Tomasz Sienkiewicz and Michał Kopiczko, for the help with my CNC milling machine parts, without which I would not have been able to make the prototype of the module.

 Mariusz Pałubicki, and Łukasz Antczak, for guidance and suggestions regarding the PCB design.

 My family, who is responsible for who I am today and who continually motivated me to pursue my passion.

 My life partner Paulina, who was directly influenced by the making of this thesis, putting up with the milling machine noise and the mess I made.

# Contents

1  Introduction 

    1.1  Motivation
    
    1.2  Thesis outline
    
    1.3  Existing actuators overview
    
    1.4  Motor module requirements
    
2  Electric motor basics

    2.1  Farady’s and Lorentz’s law
    
    2.2  Torque production and back-EMF generation
    
    2.3  Physical winding connection
    
    2.4  Total motor torque production
    
    2.5  Permanent magnet motor types
    
    2.6  Torque production
    
3  Control strategy
    
    3.1  Introduction
    
    3.2  Sensored and sensorless control
    
    3.3  Brushless motor reference frames 
        
        3.3.1  Stator reference frame
        
        3.3.2  Rotor reference frame
        
    3.4  Brushless motor control algorithms 
    
        3.4.1  Six step / trapezoidal / squarewave
        
        3.4.2  Sinusoidal
        
        3.4.3  Field Oriented Control (FOC)
        
        3.4.4  Direct Torque Control (DTC)
        
        3.4.5  Control techniques summary 
     
4  Actuator design
    
    4.1 Control algorithm design and simulation
    
        4.1.1  PI controllers tuning
        
        4.1.2  Simulation results
    
    4.2  Mechanical
    
        4.2.1  Motor selection
        
        4.2.2  3D model
        
        4.2.3  Manufacturing the actuator 
        
    4.3  Electrical
    
        4.3.1  Controller electrical requirements
        
        4.3.2  Schematic and PCB design
        
    4.4  Software
    
        4.4.1  Commutation routine
        
        4.4.2  Timer
        
        4.4.3  ADC measurements
        
        4.4.4  Position and velocity measurement
        
        4.4.5  Encoder calibration routine
        
        4.4.6  Current measurement calibration 
        
        4.4.7  Automatic motor parameters identification
        
        4.4.8  High level control
        
        4.4.9  CANFD communication
        
        4.4.10  Bootloader
        
        4.4.11  Updating the firmware 
        
        4.4.12  Control application
        
5  Results

    5.1  Initial waveform validation
    
    5.2  Torque transducer test bench
    
    5.3  Thermal testing
    
    5.4  Parameter identification and decoupling
    
    5.5  High level control
    
    5.6  Summary and future work
    
Literature

Appendix 

# 1 INTRODUCTION

Nowadays, mobile robots are able to fulfil incredible tasks – from autonomous highspeed air maneuvers to diving in the deepest parts of oceans. They are used in harsh environments, normally too dangerous or unavailable for people. Mechanical structures of such robots and their control algorithms are often inspired by their biological counterparts – animals. However, for a long time this was not the case for legged robots. They were really slow, making a single step at a time, which was not useful in real life conditions, considering the speed at which the locomotion occurred. Currently, the most advanced walking robots present the ability of walking in difficult terrains such as forests, rocks, and steep inclines. They are capable of performing jumps, rapid rotation, high speed running, or even a 360° backflip motion [1]. These dynamic maneuvers bring robots’ abilities closer to motions earlier known only from observing animals. Crucial elements to all of these abilities are actuators. High torque density, low mass, high output velocity, and low output inertia are only a few of the many requirements imposed on modern actuators used in walking robots. These devices have to be capable of not only producing torque but also reading the reaction torque acting from the outside environment [2]. In most cases, it means the actuator has to be back-drivable. All these features usually result in a high cost of a single walking robot actuator.

In this thesis, a small, brushless actuator for dynamic legged robots is presented. Compared to professional solutions, the module is inexpensive, due to the usage of easily accessible parts such as brushless drone motors and gear system acquired from an electric drill. Preserving a low cost factor, the actuator can still present high torque density, high velocity operation, and torque sensing capability, thanks to an insightful analysis and usage of available solutions and off-the-shelf materials.

## 1.1 Motivation

People have studied legged animals’ movements for ages and tried to mimic their movements with machines [3]. Some of them were designed to be purely mechanical [4], whereas the others used some type of actuation – either electric or hydraulic. However, none of them would move in the way animals do. This was caused by the assumption that the robot has to be statically stable at all times during the movement. Moving the center of the robot’s mass from one support polygon to the other would take very long, resulting in slow movement. To ensure stability, walking robots were equipped with many legs and low center of gravity (Figure 1-1). Watching these robots move, one can instantly notice their gait looks very unnatural.

Figure 1-1 Six-legged walking machine (Institute Vniitransmash 1979) [I1]

This trend of statically stable walking was interrupted by Marc Raibert’s contribution to this subject. During his work at Massachusetts Institute of Technology (MIT), he developed a series of dynamic legged robots. Some of them were single leg “choppers” (Figure 1-2), the others were two or even four-legged chopping machines.

Figure 1-2 Marc Raibert's single leg chopper (Massachusetts Institute of Technology, 1983) [I2]

All of them presented a rather unusual type of movement – instead of safe and slow walking patterns, these robots would jump and run at high speeds. All this was accomplished with very little computational effort due to the Raibert’s heurisitcs [5] that he invented at that time. Boston Dynamics, currently the most popular robotics company in the world, is actually MIT’s spin-off created by Marc Raibert. Initially, they were aiming for military robots able to carry heavy equipment in the battlefield. Right now, they are implementing their robots in more civilian applications. Spot is the first commercial quadruped robot engineered by Boston Dynamics, that was released in June 2020. Shortly after the release, the robot was used as a supervisor on the SpaceX launch pad [I3], observing the main tank for possible flaws during refueling with highly explosive rocket fuel. During the coronavirus outbreak, Spot robots patrolled the Singapore’s parks and reminded people about the social distancing rules if they did not obey them [I4]. Later on, in November 2020, New York Police Department used Spot robot in a manhunt event, when a shooter barricaded himself in one of the basements near the crime scene [I5]. All these situations show that there is a need for agile and dynamic robots able to fulfil supervisory tasks in difficult (from the robot’s point of view) environments.

Spot robot can be equipped with a robotic arm which allows it to perform manipulation tasks such as moving objects, or even opening doors (Figure 1-3). This can be thought of as a manipulator arm with an infinite workspace, which opens many new possibilities. The robot is able to automatically detect and climb stairs as well as perform different types of gait adjusted for the type of terrain it is currently on. Spot, in its early stages of development, was already a great source of inspiration for many companies as well as individuals all over the world.

Figure 1-3 Spot robot performing manipulation task [I6]

Moreover, research dedicated to quadrupedal robots is carried out by some of the world’s most prestigious universities such as MIT (USA, Boston) [1], ETH (Switzerland, Zurich) [6], or Stanford University (USA, California) [7] (Figure 1-4). Robots created in these universities’ laboratories have repeatedly proven their great agility and robustness, sometimes even exceeding animals’ capabilities. Many research materials created during the process of investigating the most efficient solutions for walking robots have substantially enriched the state of knowledge.

Figure 1-4 a) Doggo robot (Stanford University) [I7], b) HyQ robot (Istituto Italiano di Tecnologia (IIT))[I8], c) ANYmal C robot (ANYbotics company (ETH spin-off))[I9]

There is also a growing trend of interest in legged robots among makers and hobbyists, who share their projects on the Internet. Considering the fact that walking robots are incredibly complex structures, in both hardware and software aspects, building one is a challenging intellectual task. The subject is connected with many different fields of science – control theory, electric/magnetic circuits, programming, optimization, EMC (ElectroMagnetic Compatibility) related issues, and many more, so it is a great practical exercise for gaining knowledge in these disciplines.

All these facts make the author believe that legged robots are a futuristic and innovative direction that is worth exploring.

## 1.2 Thesis outline

The aim of this thesis is to design, build, and test a small brushless actuator for application in dynamic legged robots. First, the already existing solutions are presented and their main features are analyzed. Then, a short theoretical introduction is carried out to derive the most important equations governing brushless motors. In the next chapter, a comparison of available control techniques is presented and the most optimal algorithm is chosen. The algorithm is then implemented in a simulation environment to validate its performance. Afterwards, both electrical and mechanical aspects of the designed actuator are described. The actuator 3D model is shown in the form of an exploded view, while the PCB is analyzed layer by layer. In the end of this chapter, software aspects such as main control routine and bootloader logic are described in detail. Finally the performance of the module is presented, as well as future work and possible actuator improvements.

## 1.3 Existing actuators overview

Although quadrupedal robots are still a fairly new field of research, there are some high end actuator solutions available from all over the world. Before designing a custom actuator, it is essential to investigate each of them and choose the optimal components and algorithms in order to maintain similar performance. This overview will focus on both amateur as well as professional solutions, presenting their main features.

a) **Unitree A1 actuator** – modern actuator optimized for usage in dynamic walking robots. It was designed by a Chinese company Unitree involved in quadrupedal robot research since 2013. This 600 g actuator is able to produce up to 33.5 Nm peak torque. The motor is specially designed for low cogging torque, high positioning accuracy, and high transmission transparency. The module is claimed to be shock, water and dustproof. According to the manufacturer’s statement, the gearbox is equipped with cross-roller bearings that can withstand axial loads, instead of the commonly used radial ball bearings. In case of excessive heating, the case can be equipped with an additional radiator for improved heat dissipation. Photos included on the product website show that the rotor has a concave shape in order to make the actuator even more packed. The maximum joint velocity is 21 rad/s at 24 V. The company published numerous video clips on highly dynamic manoeuvres presented by the robot equipped with A1 actuators.

Figure 1-5 Unitree A1 actuator [I10]

b) **ANYdrive actuator** – an actuator designed and manufactured by ANYbotics company in Zurich. This actuator uses mechanical compliance in the form of a stiff spring placed in series with the gearbox and output shaft. The spring serves as temporary energy storage and protection in case of high velocity impacts. Moreover, the deflection of the spring is measured and converted to a torque value. This approach allows for high impact robustness (compared to rigid actuator structures), although at the cost of reduced maximum torque bandwidth. The actuator weights 900 g and is able to produce up to 40 Nm of peak torque.

Figure 1-6 ANYdrive actuator [I11]

c) **Mini Cheetach actuator** – an amateur actuator designed by a former MIT student, Benjamin Katz, as a part of a master’s thesis project [8]. The actuator is based on a largediameter drone brushless motor combined with a planetary gear reducer placed in the stator. This unusual placement of the gear system resulted in actuator length reduction. The motor module is driven by a custom controller utilizing field oriented control. Output torque is estimated by measuring the phase currents, which is possible due to the low gear ratio (6:1). It is capable of producing up to 6.9 Nm of continuous torque and 17 Nm peak torque, weighing approximately 480 g. Maximum output velocity is 40 rad/s at 24 V. The actuator was used in MIT Minicheetach quadrupedal robot that was the first four-legged robot to perform a 360° backflip.

Figure 1-7 Minicheetach actuator [8]

d) **INNFOS QDD NU-80-6** – a commercial version of Minicheetach actuator with a similar form factor, able to achieve 6.6 Nm continuous and 19.8 Nm peak torque, weighing 453 g. The maximum speed is 38 rad/s. Similarly to previous actuators, it is equipped with a single stage 6:1 gear ratio planetary reducer. The main application of the module is in quadrupedal robots. Smaller brushless actuators are also available in the company’s catalog (52 mm diameter), however they feature high ratio gear reducers (30:1) which result in low output speeds (about 6 rad/s), inefficient for legged locomotion.

Figure 1-8 NU-80-6 actuator [I12]

## 1.4 Motor module requirements

Actuators used in legged robots have to meet some specific requirements. One of the most important factors is the torque density, expressed as the torque carrying ability of a system in a given space and weight envelope. This factor is of great importance as actuators are the majority of the robot’s overall mass. That is the reason brushless motors are used, as they have superior torque to mass ratio. High torque production ability is another important issue. Robot’s legs have to be able to produce rapid forces to counteract the disturbances from the environment. Occasionally, it is necessary to produce high torque for a short period of time, for example, when jumping or trotting. Concurrently, the motor should remain in the safe temperature range to prevent overheating.

One can notice that animals’ joints can be rotated not only by their muscles, but also by the external forces acting on the limbs. A similar approach is being adopted in most of the modern walking robots - a high transmission transparency, that allows for implementing compliance in the joints. Transmission transparency is described as a measure of how easily the energy flows from the actuator to the end effector in both directions [I13]. In the case of a transparent system, the gearbox and the motor will not get damaged when a sudden, unexpected external impact occurs within a certain velocity limit. As one might expect, it is utilized by applying easily backdrivable, low ratio gearboxes between the motor and the end effector, and low inertia components.

Being able to sense limb reaction forces is inevitable in advanced walking robots. One way to implement force sensing is to use a strain gauge located in the robot’s foot. This, however, adds complexity to the system and makes the leg more fragile and prone to failure from mechanical stroke. That is why researchers commonly use already available structural elements – the actuators. Each brushless actuator driven with vector control algorithm has the ability to read the output torque based solely on motor phase currents. This method can introduce some inaccuracies, especially when the motor is accelerating, but at the same time is very fast and cost-efficient. It requires the motor to be easily backdrivable (high transmission transparency is essential) so that the response is fast and not influenced by inertia or friction in the gearbox.

# 2 ELECTRIC MOTOR BASICS

Before moving on to the motor characterization procedure and selecting the right motor, it is vital to describe the basic rules governing the electric motors.

# 2.1 Farady’s and Lorentz’s law

The first considered rule is the Faraday’s law of induction:

>    𝑒(𝑡) = −  𝑑φ 𝑑𝑡  (2.1)

It describes the relation between the change of magnetic flux φ in time, linked by a coil (single turn of wire) and the voltage induced in that coil. The magnetic flux is the amount of field lines going through a surface and is expressed as surface integral:

>    𝑛̂ 𝑑𝐴 φ = ∫𝐵  (2.2)

⃗where 𝐵⃗ is the magnetic field vector and 𝑛̂ is a unit vector perpendicular to the coil’s surface where 𝐵 ⃗ and the 𝑛̂ vector are parallel the dot product is maximized and with area 𝐴. If we assume the 𝐵 reduces to scalar multiplication. In order to further simplify the expression (2.2), we can assume ⃗ the coil has a rectangular shape and is being inserted into a constant magnetic field 𝐵 perpendicular to the surface of the coil (Figure 2-1). By varying the frame position one can vary the amount of field lines through a surface, i.e., the magnetic flux.

Figure 2-1 A copper frame inserted into a constant magnetic field

Taking that into consideration and looking at Figure 2-1 the induced voltage can be ⃗ is described in terms of the rate of change of the bar position, assuming the magnetic field 𝐵 constant, and the frame is moving in the direction that increases the loop area with constant velocity v:

>    𝑒(𝑡) = −  𝑑φ 𝑑(𝐵 ∙ 𝐴) 𝑑(𝐵 ∙ 𝑙 ∙ 𝑥) 𝑑𝑥 =− =− = −𝐵 ∙ 𝑙 = −𝐵 ∙ 𝑙 ∙ 𝑣 𝑑𝑡 𝑑𝑡 𝑑𝑡 𝑑𝑡  (2.3)

The (2.3) equation is of great importance regarding the motor bEMF constant - Ke. Lorent’s law has the general form of:

>    𝐹 = 𝑞 · (𝐸⃗ + 𝑣 ⨯ 𝐵  (2.4)

where 𝐹 is the force acting on a particle q moving in the electric field 𝐸⃗ and magnetic field 𝐵 with velocity 𝑣. However, when describing electric motors, it can be significantly simplified due to the few constraints imposed on these machines. First, we can assume the effect of electric ⃗ reduces to a scalar field 𝐸⃗ is negligible small. Moreover, the cross product of 𝑣 and 𝐵 multiplication, because of the fact that the two vectors are perpendicular to each other [1]. Further manipulation stated in [9], p.61, results in a commonly known equation: 

>    𝐹 =𝐵·𝐿·𝑖  (2.5)

where 𝐿 is the conductor length in the magnetic field 𝐵 as in Figure 2-2.
 
Figure 2-2 A conductor carrying current in a constant magnetic field experiences force

[1] It is possible to assume the perpendicualrity of these two vectors as this condition maximizes the cross product, thus maximizing the generated force which is the main goal of modern motors.

In the case of a conductor carrying current in a constant magnetic field, the generated force depends on the conductor length in the magnetic field, the current flowing through the ⃗ . Usually the magnetic field is constant and the conductor has a circuit, and magnetic field 𝐵 fixed length, thus the force is directly proportional to the current.

Equation (2.5) is going to be crucial when describing the motor’s torque constant Kt.

## 2.2 Torque production and back-EMF generation

The torque generated by a simple single-phase two-pole motor can be described with the following equation:

>    𝑇(𝛩) = 2 · 𝐹(𝛩) ·  𝐷 = 𝑁 · 𝐵(𝛩) · 𝐿 · 𝑖 · 𝐷 2  (2.6)

where 𝐹 is the tangential force generated by a single coilside, 𝐷 is the diameter of the coil, 𝑁 is the number of wire turns, 𝑖 is the current, and 𝐵 is the magnetic field. One has to remember that 𝐵 is actually a function of the rotor position, depending on the flux distribution in the airgap of the motor, as it is per phase model [2]. In the case of a sinusoidal motor, based on (2.6), a torque constant 𝐾𝑡 can be introduced as:

>    T(𝛩) = 𝐾𝑡 · sin(𝛩) · 𝑖(𝑡)  (2.7)

[2] When dealing with multiphase sinusoidal motors, the resultant torque is going to be much less position dependent as adding phases results in lowering the torque ripple, simultaneously increasing its frequency.

The unit of the torque constant is [Nm/A].

The (2.3) equation is used to determine the bEMF constant:

>    𝑒(𝛩) = 2 · 𝑁 · 𝐵 · 𝐿 · 𝑣 = 2 · 𝑁 · 𝐵(𝛩) · 𝐿 ·  𝐷 · 𝜔 = 𝑁 · 𝐵(𝛩) · 𝐿 · 𝜔 · 𝐷 2  (2.8)

where 𝜔 is the rotational velocity of the rotor. Again in case of a sinusoidal motor, the equation can be rewritten to the form:

>    𝑒(𝛩) = 𝐾𝑒 · sin(𝛩) · 𝜔(𝑡)  (2.9)

where Ke is the bEMF constant expressed in [volt·s/rad].

What is essential to note is that the Ke and Kt constants both are numerically equal when using SI units. They share a common unit of [kg·m2·s-2·A-1]. This relation can be easily proven as in (2.10), by comparing the electrical and mechanical power of a motor:

>    τ·𝜔 =𝑒·i  (2.10)

which can be further transformed to:

>    [𝐾𝑡 · i] · 𝜔 = [𝐾𝑒 · 𝜔] · i  (2.11)

This essential relationship allows for quickly finding the estimates of motor’s constants based on a simple experiment carried out in the motor comparison chapter. One has to remember that the equations presented in this section are derived based on a single phase machine. Even though the principle is the same, the torque and bEMF constants found in the motor’s datasheet are most probably not going to be numerically equal, because of the different naming conventions as well as the multiphase nature of the real motors.

## 2.3 Physical winding connection

Three phase motor windings can be connected in one of two manners – delta or wye type (Figure 2-3). Before electronic inverters were introduced, the synchronous motors were equipped with a special switch that would connect the windings in one of these two ways. This was dictated by the fact that delta-connected motors would consume high startup currents causing voltage drops in the electrical grid as well as mechanical stroke. A wye-start/delta-run method was introduced in order to reduce these undesired effects. It takes advantage of the fact that the wye connection has higher line-to-line resistance and thus the current (torque) is reduced. When the motor reaches a certain speed, the windings are switched to delta in order to reach full torque. Besides the situation when inverters were not available and the voltage was dictated by the electrical grid parameters, there is no gain in using delta connection over wye type. Additionally, the nature of delta connected motors allows for currents to circulate [9] p.201, which may lead to reduced performance or even an overheat event.

Figure 2-3 a) wye winding connection and b) delta winding connection

Throughout this thesis, only wye connection will be considered as it is the most common winding connection among all small size drone motors.

## 2.4 Total motor torque production

Previously, the torque production ability of a single phase motor was described. Torque equation for a full 3-phase sinusoidal motor can be obtained by transforming the equation for energy balance at any given time instant:

>    𝜏 ∙ 𝜔 = 𝑒𝐴 ∙ 𝑖𝐴 + 𝑒𝐵 ∙ 𝑖𝐵 + 𝑒𝐶 ∙ 𝑖𝐶  (2.12)

Solving the equation (2.12) for τ, the equation (2.13) can be written as:

>    𝜏=  𝑒𝐴 𝑒𝐵 𝑒𝐶 ∙ 𝑖𝐴 + ∙ 𝑖𝐵 + ∙ 𝑖𝐶 𝜔 𝜔 𝜔  (2.13)

If we assume the motor is balanced, i.e., all three phases are identical and we substitute 𝑒  each 𝜔 term with 𝐾𝑒 ∙ sin(𝜃) from equation (2.9) the equation transforms to:

>    𝜏 = 𝐾𝑒 ∙ sin(𝜃) ∙ 𝑖𝐴 + 𝐾𝑒 ∙ sin(𝜃 + ) ∙ 𝑖𝐵 + 𝐾𝑒 ∙ sin(𝜃 − ) ∙ 𝑖𝐶 3 3  (2.14)

Each phase current is a function of 𝜃 angle and has to be in phase with bEMF waveforms. With that being said, the equation (2.15) is evaluated.

>    𝜏 = 𝐾𝑒 ∙ sin(𝜃) ∙ 𝑖 ∙ sin(𝜃) + 𝐾𝑒 ∙ sin (𝜃 + ) ∙ 𝑖 ∙ sin (𝜃 + ) 3 3  (2.15)
>    𝜋 𝜋 +𝐾𝑒 ∙ sin(𝜃 − ) ∙ 𝑖 ∙ sin(𝜃 − ) 3 3 𝜋  𝜋  

Using trigonometric identities expression [sin2 (𝜃) + sin2 (𝜃 + 3 ) + sin2 (𝜃 − 3 )] reduces to 3 2  constant value and the equation (2.15) is reduced to (2.16). The graphical explanation of  equation (2.15) is depicted in Figure 2-4.

Figure 2-4 Drive currents and bEMF waveforms vs torque production in sinusoidal motor

>    𝜏=  3 ∙𝐾 ∙𝑖 2 𝑒  (2.16)

A similar approach can be taken when dealing with arbitrary trapezoidal motors. Based on Figure 2-5 one can notice that only two phases are energized at any given moment and contribute to overall motor torque production [3]. Thus, the resultant torque can be estimated with (2.17) formula.

[3] This is true only for ideally square bEMF and current waveforms, that are impossible to achieve on real hardware. Trapezoidal motors use to have longer transition states and thus less than 120° of flat top waveform. These transition segments contribute to torque variations.

Figure 2-5 Drive currents and bEMF waveforms vs torque production in trapezoidal motor

>    𝜏 = 2 ∙ 𝐾𝑒 ∙ 𝑖  (2.17)

It is crucial to notice that the torque equation (2.17) involve 𝐾𝑒 constant which is defined in the single phase model as the line-to-neutral quantity. In real motors, however, a common winding connection is wye type with isolated (floating) neutral connection that is not provided outside the motor case as shown in Figure 2-6. This is why motor constants found in datasheets documents are commonly described as line-to-line values. The conversion between line-to-line and line-to-neutral quantities is rather straightforward, although it depends on the motor type being examined.

Table 2-1 Comparison of trapezoidal and sinusoidal motor parameters

Parameter

Sinusoidal

Trapezoidal

Overall torque constant defined with line-to-neutral 𝐾𝑒 constant

Line-to-line 𝐾𝑒 constant - 𝐾𝑒𝑙𝑙

Overall torque constant defined with line-to-line 𝐾𝑒 constant

>    𝐾𝑇 =  3 ∙𝐾 2 𝑒
>    𝐾𝑒𝑙𝑙 = √3 ∙ 𝐾𝑒
>    𝐾𝑇 =  √3 ∙ 𝐾𝑒𝑙𝑙 2
>    𝐾𝑇 = 2 ∙ 𝐾𝑒 
>    𝐾𝑒𝑙𝑙 = 2 ∙ 𝐾𝑒 
>    𝐾𝑇 = 𝐾𝑒𝑙𝑙  

Figure 2-6 𝐾𝑒 and 𝐾𝑒𝑙𝑙 definition

## 2.5 Permanent magnet motor types

It is essential to note that there are two most commonly known types of permanent magnet brushless motors, distinguished by the shape of their bEMF waveforms – BLDC (Brushless DC motor) and PSMS (Permanent Magnet Synchronous Motor). Usually the two types are very much alike from the mechanical point of view – it is impossible to determine the motor type just by visual aspects. The difference between the two is caused by the winding style, stator teeth construction, magnet dimensions, and their placement on the rotor’s surface. As could be foreseen, the two types differ in control algorithms, as for reaching maximum torque the controller has to keep the current in phase with bEMF waveform. Figure 2-7 depicts the difference in bEMF waveforms for each type of motor. In the literature, BLDC motors are frequently mistaken with PMSM motors, which is the reason for a lot of confusion in the terminology. The general rule is that trapezoidal motors are referred to as BLDC, whereas sinusoidal are called PMSM [10]. This naming convention will be used throughout this report.

Figure 2-7 Flux distribution (bEMF shape) in a) PMSM motor and b) BLDC motor

Projects of other walking robots and walking robots’ actuators seem to choose PMSM over BLDC motors and there is a good reason for that. A comparison carried out in [11] shows differences in both motor structures and their applications. Trapezoidal motors are used in machines requiring high speed operation and continuous rotation. They are cost efficient and do not require advanced drivers. However, these features come at a price of higher torque ripple, which can eliminate the possibility of accurate position control and force estimation, especially at low speeds. On the other hand, PMSM motors are characterized as a much better solution for position control, due to the sinusoidal shape of the driving currents, which results in smooth operation with negligible torque ripple. They have superior power density (comparable to stepper motors) and are able to reach reasonably high speeds. Due to the fact that legged locomotion requires discontinuous rotation and smooth torque control, it is justified to choose the sinusoidal motor over the trapezoidal one. The bEMF waveform shape of an externally rotated motor is a good measure for characterizing the motor between the two types if it is unknown.

## 2.6 Torque production

Torque produced by the motor depends on many different structural parameters. It can be easily determined that commercial legged robot actuators are commonly equipped with flat, large diameter motors. This is due to the fact that a larger air gap radius [4] motor results in greater torque density, whereas an axially longer motor does not contribute to the increase in torque density as it is equivalent to adding identical motors on the same axle [2]. This implies it is better to choose a motor with a larger air gap radius instead of an axially longer motor, assuming the mass is fixed. Outrunner motors are preferred based on this criterion, as they have a larger airgap in comparison to inrunner motors with the same outer diameter.

[4] Air gap radius (Rgap) is the parameter which expresses the length from motors center to the center of motor’s airgap. It is commonly used for describing brushless motors [2]

Figure 2-8 A schematic view of a) inrunner and b) outrunner motor

A common discussion is conducted about different KV motors. KV is a parameter used by hobby drone motor manufacturers and is expressed in rpm/V units. This can be thought of as a speed constant (line to line quantity). The KV parameter may be useful when characterizing motors and estimating their torque constant. Based on (2.16), it is possible to derive:

>    𝐾𝑡 =  3 1 60 1 ∙ ∙ ∙ 2 √3 2𝜋 𝐾𝑉  3  


where term 3/2 indicates the motor is sinusoidal (PMSM), term 1/√3 is used for converting line to line to phase quantities and 60/2𝜋 is used for converting units from rpm to rad/s. This rough estimation can be really useful when only KV parameter is known. However, the analysis based on KV parameter alone is not a good measure of the motor’s ability to produce torque when real life limitations are imposed, such as limited voltage or current. Assuming an ideal inverter, there should not be any difference in performance among motors with different KV values (and thus torque constants), as long as the amount of copper on the stator’s teeth is the same. However, when the inverter is voltage or current limited, there may be the possibility of not being able to reach full power. When the phase resistance is too high (many turns of thin wire, low KV), high voltages are needed to push the amount of current needed to produce the same amount of torque as in the motor with higher KV, but lower phase resistance. On the other hand, when the resistance is too low (few turns of thick wire, high KV), high currents are needed to produce the same amount of torque (as there are less turns affected by the magnetic field), in comparison to a motor with lower KV, but higher phase resistance. In case of reaching either limit, it is easier to increase the current capability than the maximum allowed voltage for the motor controller. This is usually done by replacing MOSFET transistors with higher maximum current ones and increasing the DCBUS capacitance. However, increasing the maximum voltage that can be handled, is not as straightforward due to the component’s limitations such as maximum MOSFET driver voltage, maximum drain-source MOSFET voltage, ceramic capacitors maximum voltage and capacitance derating. Moreover, higher voltage is more expensive in terms of remote power supplies – the batteries. Higher cell number battery pack is usually more expensive and requires a specialized charger. This is why lower resistance motors are preferred over higher resistance ones. It is worth noting that two mechanically similar motors with different phase resistances can present torque constants different by even an order of magnitude. Consequently, motors should not be compared with torque constant, but motor constant that is expressed as:

>    𝐾𝑚 =  𝜏 √𝑃  =  𝐾𝑡 ∙ 𝐼 √𝐼 2 ∙ 𝑅  =  𝐾𝑡 √𝑅  (2.19)

where τ is torque, 𝑃 is power, and 𝑅 is line to line resistance. Motor constant should be used to compare motors between each other as it takes into account the motor resistance.

Torque capability is also influenced by the number of motor pole pairs as described in:

>    𝜏=  3𝑝 ∙λ ∙𝑖 2 𝑟 𝑞  (2.20)

where p is the number of motor poles, λ𝑟 is the permanent magnet flux linkage and 𝑖𝑞 is current in q axis (which is going to be explained later on). It seems obvious that motor with higher pole count should be able to produce more torque than motor with a lower pole count. This can be misleading as higher pole number stator has less volume for copper wire, which is directly responsible for torque production. As a result, each tooth of a higher pole number motor can be wound with fewer wire turns. Therefore, motor pole number should not be the only factor when comparing torque capabilities of different motors.

On the other hand, increasing flux linkage is always beneficial for torque production. This is why it is preferred to choose strong rare-earth magnets instead of ceramic or ferrite magnets.

# 3 CONTROL STRATEGY

## 3.1 Introduction

Brushless motors, in contrary to DC brushed motors, are not equipped with a mechanical commutator. This makes them more durable, however, it also means the responsibility for energizing the appropriate windings rests on the electronic motor controller. The controller has to utilize the most appropriate control algorithm for a specific application it is used in. There are a lot of brushless motor control techniques, although the legged robot application requires some specific features. The actuators are expected to produce full torque at zero speed and present dynamic behavior and smooth performance over a wide range of speeds. The controller should keep track of the rotor position and respond accordingly to keep the desired trajectory even if the motor stops unexpectedly or is heavily loaded. Moreover, sufficient torque bandwidth is essential for agile locomotion, reacting to pushes and disturbances, which may occur while walking. The comparison carried out in this section will allow for choosing the best possible control algorithm.

## 3.2 Sensored and sensorless control

Finding the position of the rotor, which is usually indispensable for proper motor control, can be performed in two ways – using external position sensors (sensored) or by sampling the bEMF voltage signal induced in the unenergized winding (sensorless). The sensorless method is used in applications requiring continuous high speed operation, as at lower speeds the bEMF tends to be too small to measure. At the expense of a more complicated control algorithm, it is possible to omit the external costly sensor. Even though, low speed operation and startup are going to suffer from inaccurate position estimation.

 External encoders are the solution for inaccurate positioning. They feature high positioning accuracy and allow for superior speed measurements, when needed for high level control algorithms. Supposedly, their biggest disadvantage is high cost. Moreover, if magnetic sensors are used, the magnet and the sensor should be aligned with high precision. Otherwise, the error should be measured and accounted for when reading position from the sensor. If not compensated for, there might be some read position fluctuations leading to additional motor torque ripple.

Taking these facts under consideration, it was decided to use an external position sensor to mitigate any effects of inaccurate position determination.

## 3.3 Brushless motor reference frames

A brushless motor can be analyzed in both the stator and the rotor reference frame.

### 3.3.1  Stator reference frame

The stator reference frame assumes the stator is stationary and all quantities are expressed with respect to it. Each axis is aligned with a single motor phase. A schematic picture of this coordinate frame is depicted in Figure 3-1.

Figure 3-1 Stator reference frame

The starting point for evaluating the three phase motor equations is the simple single phase voltage equation:

>    𝑉 = 𝑅𝑖 + 𝑑𝑡 𝜆 + 𝑒  (3.1)

where 𝑅 is the phase resistance, 𝑖 is the current, 𝜆 is the flux linkage, and 𝑒 is bEMF voltage. In terms of a 3 phase motor, the equation can be represented in a matrix form as follows:

>    𝑉𝐴 𝑅 [𝑉𝐵 ] = [ 0 𝑉𝐶 0  0 𝑅 0  𝐿𝐴𝐴 0 𝑖𝐴 0 ] [𝑖𝐵 ] + [𝐿𝐴𝐵 𝐿𝐴𝐶 𝑅 𝑖𝐶  𝐿𝐴𝐵 𝐿𝐵𝐵 𝐿𝐵𝐶  𝑒𝐴 𝐿𝐴𝐶 𝑑 𝑖𝐴 𝐿𝐵𝐶 ] [𝑖𝐵 ] + [𝑒𝐵 ] 𝑒𝐶 𝐿𝐶𝐶 𝑑𝑡 𝑖𝐶  (3.2)

where 𝐿 with a lower subscript describes either the self (identical letters) or mutual (different letters) inductances. Designing a current controller for this representation is not a trivial task as the voltages must vary depending on the rotor position to produce a constant torque. This is why it is beneficial to transform to a nonstationary frame – the rotor frame.

### 3.3.2  Rotor reference frame  The rotor reference frame is fixed to the moving rotor. There are two axes – d (direct) aligned with the direction of the magnetic flux, and q (quadrature) which is rotated 90° electrical from the d axis (Figure 3-2).

  Figure 3-2 Rotor reference frame Transforming the 3-phase quantities to this reference frame makes it much easier to analyse the motor equations and directly control the torque. This is due to the fact that in this frame the currents are rotor position invariant. The transition between coordinate systems is made with Clark (equation (3.3)) and Park (equation (3.4)) transformations [12].The first transformation is used to describe three phase stationary quantities with two orthogonal components, whereas the second fixes the coordinates to the moving rotor.

 1 1 − 2 2 √3 √3 − 2 2 1 1 √2 √2 ]  (3.3)  cos(θ) sin(θ) 0 𝑇𝑃 = [−sin(θ) cos(θ) 0] 0 0 1  (3.4)  1 𝑇𝐶 =  √3 2  −  0 1 [√2  30 32:1014306560   The transformed voltage equations in d-q frame are presented as follows [13]: 𝑉𝑞 = 𝑅𝐼𝑞 + 𝐿𝑞  𝑑𝐼𝑞 + 𝜔𝜆𝑟 + 𝜔𝐿𝑑 𝐼𝑑 𝑑𝑡  (3.5)  𝑑𝐼𝑑 − 𝜔𝐿𝑞 𝐼𝑞 𝑑𝑡  (3.6)  𝑉𝑑 = 𝑅𝐼𝑑 + 𝐿𝑑  where 𝑉𝑞 , 𝑉𝑑 are the voltages in d/q axis, 𝑅 is the phase resistance, 𝐿𝑞 is quadrature axis inductance, Ld is direct axis inductance, 𝜔 is the electrical angular speed and 𝜆𝑟 is the total rotor flux linkage. The bEMF term is shown here in terms of the rotational velocity. In contrast to stationary coordinate system, this representation causes the torque to be constant when the 𝐼𝑑 and 𝐼𝑞 values are constant. This greatly simplifies the current control algorithm and allows to use PI controllers for each axis. Before PI controllers are implemented on each axis, coupling terms linking both (3.5) and (3.6) equations should be mitigated. It is done by introducing feedforward terms [14]: 𝑉𝑞 𝑓𝑓 = −𝜔𝐿𝑑 𝐼𝑑  (3.7)  𝑉𝑑 𝑓𝑓 = 𝜔𝐿𝑞 𝐼𝑞  (3.8)  Torque equation described with quantities expressed in rotating d-q frame can be described as follows: 𝜏=  3𝑝 𝑖 (𝜆 + 𝑖𝑑 (𝐿𝑑 − 𝐿𝑞 )) 2 𝑞 𝑟  (3.9)  The equation (3.9) is general and it includes the eventual difference in d and q axis inductances (in case of salient motors [10]) that contributes to reluctance torque. In this thesis, only surface mount permanent magnet motors are considered, so the equation can be reduced to (2.20).

  3.4 Brushless motor control algorithms Brushless motor control algorithms can be divided into a two categories – scalar and vector control. Scalar control bases on the idea that only the voltage magnitude and frequency can be varied directly, and it is the easiest strategy to implement. On the other hand, vector control allows for a higher level control (torque, magnetic flux), which is much more complex and offers certain advantages in comparison to the scalar methods.

 3.4.1  Six step / trapezoidal / squarewave 31  33:1011624895   This algorithm is the easiest one to implement, as it is based on changing the energized windings based on the rotor position, whereas the third winding is always left undriven. It can be utilized with hall-effect sensors, serving as a low resolution rotor position sensor, or with sensorless position estimation. It does not require a lot of computational power, however, suffers from torque ripple due to rapid winding switching instants. Usually, this technique is implemented in continuously rotating motors. The amplitude of the waveforms can be controlled by varying DC bus voltage or PWM modulation.

 3.4.2  Sinusoidal  PWM modulation allows for creating an arbitrary voltage waveform. The sinusoidal technique is implemented on inverters working with PMSM motors. Driving a sinusoidal motor with sinusoidal waveform is beneficial as the resultant torque is smooth and the torque ripple is minimized. This technique is usually implemented with hall sensors, but high resolution encoders can be also used. Similarly to six-step/trapezoidal algorithm, when Hall sensors are utilized, the motor is susceptible to stall events and cannot operate at low speeds, as the rotor position is only updated every 60° electrical, and between the Hall sensors the position is estimated. Although using a high resolution encoder is beneficial due to the high frequency position information it is able to output.

 3.4.3  Field Oriented Control (FOC)  The FOC algorithm (sometimes also referred to as “vector control”) is based on transforming the stator-fixed three-phase quantities to two orthogonal components fixed to the rotating rotor. This representation allows for direct torque and magnetic flux control with just two parameters (current in q and d axis). Moreover, the torque produced by the motor is proportional to q axis current, when operating in the linear range, where no saturation effects are present. The d axis current reference is usually kept at zero, in order to maximize the torque production. The FOC algorithm is superior to the methods presented earlier, however, at a cost of additional computational cost and higher system complexity.

  32 34:1140922064    Figure 3-3 Field oriented control scheme A general field oriented control scheme is shown in Figure 3-3. Two reference currents in the d and q axis are commanded by the high level controller and are compared with measured current values. The two PI controllers evaluate new control voltages. Feedforward terms are included in the control loop to decouple the two axes and make them independent. Calculated voltages are transformed by inverse Park and then inverse Clark transforms. Afterwards, the sinusoidally varying voltages are fed into the SVPWM [15] modulator5. In the last stage, the voltages are approximated with the hardware PWM timer. The main feedback loop performs forward Clark and Park transformations on the measured phase currents in order to calculate the real d and q axis currents. The rotor angle, essential for the transformations, is obtained with low latency magnetic encoder. As stated earlier, the d axis current reference is usually set to zero, however, when bEMF is the limiting factor for reaching high speeds, a nonzero d axis current can be commanded to decrease the magnetic field strength, and thus the bEMF. This is the, so called “field weakening” action and it allows for reaching greater rotor velocities at a cost of lower torque. This technique works best on high inductance motors, thus when the inductance is low, a lot of current is needed to influence the magnetic field, which results in high Joule heating losses.

 3.4.4  Direct Torque Control (DTC)  The DTC control strategy is somewhat similar to FOC, however it does not require coordinate transformation nor PWM control. It is based on the idea of rapidly applying voltage 5  The SVPWM (Space Vector PWM) modulator is used to make the linear modulation range wider by about 15% in comparison to Sine PWM (SPMW) modulation. In result the DC bus is better utilized, the THD (Total Harmonic Distortion) is reduced and performance near overmodulation region is improved.

  33 35:1778655388   vectors based on the setpoint torque and magnetic flux values [16]. The measured torque and magnetic flux values are estimated from the stator currents and the actual voltage vector. The DTC technique does not require a position sensor and thus is considered sensorless. It is easier to implement, however, suffers from poor low speed operation and increased torque ripple. The control scheme is presented in Figure 3-4.

  Figure 3-4 Direct torque control scheme 3.4.5  Control techniques summary  The choice of the most suitable algorithm was dictated by the requirements imposed on the designed actuator. Smooth low speed operation, low torque ripple, and full torque at a wide range of speeds were the most important factors. Looking at the classification above, it is obvious that the scalar control algorithms were rejected due to their poor low speed operation and torque control. Due to poor low speed operation and increased torque ripple, DTC was also rejected. The FOC algorithm was chosen as it has superior performance in this application in comparison to other described methods.

  34 36:7649554018    4 ACTUATOR DESIGN 4.1 Control algorithm design and simulation Prior to implementing the control strategy on a real hardware module, the algorithm was tested in a simulation environment. GeckoCIRCUITS software [17] was chosen as it is made especially for simulating power electronics. It is equipped with many electrical models such as MOSFET transistor or PMSM motor model. Pieces of control algorithm are written in JAVA scripts and put into functional blocks with inputs and outputs. Signals are used to connect different algorithm blocks to allow data exchange between them. Finally, the outcome of the simulation can be presented in graphical form using the scope widget.

  Figure 4-1 Simulation schematic The electrical part of the motor controller simulation consists of a DC bus power supply, DC bus capacitors, six MOSFET transistors, three shunt resistors, and the PMSM motor (Figure 4-1). Each elements’ parameters were set based on real hardware parameters in order to make it similar to the real hardware.

 Software part is divided into a few functional blocks. The low level gate control block takes in a 40 kHz triangle wave and compares it with a reference voltage coming from the main control algorithm. This block is also responsible for applying dead-time in gate control signals 35 37:1981058679   to prevent shoot-through when both the upper and lower FETs are open. Next, there is a block containing the main control algorithm – forward and inverse Clark/Park transformations, two PI controllers, one for each axis, and SV modulation routine. The resulting PWM duty cycles are fed into the previously described gate control block (Figure 4-2).

  Figure 4-2 Measurement, FOC and PWM blocks Currents are measured in specific moments – when all three low side MOSFETs are closed (exactly the middle of the 0th sector). This assures the current is not affected by PWM modulation. Separate algorithm block is used for sampling the currents by measuring the voltage drop on low-side shunt resistors. The measurements were compared to PMSM model’s internally computed phase currents to make sure they are correct. Multiple scope widgets show the simulation results – PI controller efforts, measured d/q current values, phase currents, or rotor position. The last block – spring-damper block – is responsible for calculating the reference q axis current based on the actual position, target position, velocity, spring constant, and damping coefficient (Figure 4-3).

  Figure 4-3 Current measurements, scope and high level controller blocks  36 38:1018027039   4.1.1  PI controllers tuning  The main FOC block implements two PI current controllers. For the best possible performance, it was decided to estimate the gains instead of hand-tuning the controllers. The tuning process was based on [8] and [18]. Even though the controller is implemented on a digital microcontroller, the tuning process was carried out in the s-domain, as the RL time constant is much larger (>14x) than the system’s current sampling time. A comparison to the discrete controller regarding the performance is presented in the end of this subsection. As the motor is a surface mount permanent magnet motor, the inductances in d and q axes are going to be roughly the same. This is why the process of tuning is carried out only for a single controller, and then the same gains are applied for both d and q PI controllers.

 The goal is to control the current in the d and q axes with a voltage approximated by PWM modulation. Each of equations (3.6) and (3.5) can be simplified to a simple series RL circuit (Figure 4-4), as the terms containing 𝜔 are ignored, due to much slower dynamics, when compared to the electrical RL circuit alone.

  Figure 4-4 Simplified system's model The series RL circuit can be described with a continuous transfer function: 𝐺𝑅𝐿 (𝑠) =  1 𝐿𝑠 + 𝑅  (4.1) 𝑅  It is worth noting that the RL system has a pole at 𝑠 = − 𝐿 .

 The series PI controller structure was used (4.2), as it allows for changing the loop gain only with 𝑘𝑝 gain and moving the zero location only with 𝑘𝑖 gain. This way, the zero of the PI 37 39:9895262444   controller can be placed near the RL system’s pole in order to simplify the system’s transfer function, and the loop gain can be varied independently.

 𝐺𝑃𝐼 (𝑧) = 𝑘𝑝 (1 +  𝑘𝑖 ) 𝑠  (4.2)  In order to cancel the RL system’s pole 𝑘𝑖 was derived as: 𝑘𝑖 =  𝑅 𝐿  (4.3)  Substituting the QM5006 motor measured parameters the estimated 𝑘𝑖 is equal to 2875. This results in pole cancellation that can be observed on the zero-pole plot of the open loop system (Figure 4-5).

  Figure 4-5 Pole - zero plot of the continuous open loop system – the left hand side pole is covered with PI controller’s zero Having done the cancellation, the open loop system looks like an integrator (single pole at the origin): 𝐺𝑜𝑝𝑒𝑛 = 𝑘𝑝 (1 +  𝑘𝑝 𝑘𝑖 1 )∙ ⇒ 𝑅 𝐺𝑜𝑝𝑒𝑛 = 𝑠 𝐿𝑠 + 𝑅 (𝑘 = ) 𝐿𝑠 𝑖  38 40:5840129059  𝐿  (4.4)   The next step is to calculate the 𝑘𝑝 gain. To do that, the closed loop system transfer function is obtained: 𝐺𝑐𝑙𝑜𝑠𝑒𝑑 = The  𝑘𝑝 𝐿  𝑘𝑝 𝑘𝑝 + 𝐿𝑠  (4.5)  term is substituted with 𝑝 for simplification, and 𝑠 is substituted with 𝑗𝜔 to calculate the  magnitude of system’s response at certain corner frequency. The 𝐺(𝑗𝜔) is multiplied with its conjugate and square root of this expression is taken: 1 2  1 |𝐺(𝑗𝜔)| = √𝐺(𝑗𝜔) ∙ 𝐺(𝑗𝜔)′ = ( ) 𝜔2 1+ 2 𝑝  (4.6)  Next, the magnitude is converted to dBs: 20 log(|𝐺(𝑗𝜔)|) = −10log (1 +  𝜔2 ) 𝑝2  (4.7)  The corner frequency for a closed loop system is defined as the frequency at which the frequency response is reduced by 3dBs. If we assume 𝑝 is equal to 𝜔 the (4.7) expression yields -3dBs. As 𝑝 is really a substitution for  𝑘𝑝 𝐿  then it is possible to state that:  𝑘𝑝 = 𝐿 ∙ 𝜔𝑐𝑜𝑟𝑛𝑒𝑟  (4.8)  Again for the motor used in this thesis 𝑘𝑝 is equal to 0.502 for corner frequency of 2kHz. The corner frequency can be increased, however, one must be aware of possible side effects such as audible noise due to amplified sensor noise..

 It is clear that any motor parameters identification errors contribute to change in the computed gains. Even though, the presented PI controller tuning process allows for more accurate gains estimation than it would have been by manual tuning.

 In order to validate the designed controller in time domain, the step response of the system was plotted (Figure 4-6).

  39 41:6549957033    Figure 4-6 Step response of the continuous time system As can be seen the response is very dynamic, and no overshoot is present. The setpoint is reached after roughly 500us.

 The corner frequency was validated by drawing the bode plot of the open loop system (Figure 4-7). The intersection of an open loop frequency response and the 0 dB line is equal to the open loop crossover frequency, i.e., the corner frequency of the closed loop system [19].

  Figure 4-7 Continuous time open loop bode plot 40 42:7851332923   By analyzing the frequency and phase response of the open loop system, it is possible to find the stability margins of the closed loop system. In this case, the gain margin is infinite (the phase never reaches -180°) and the phase margin is equal to 90°. The crossover frequency is equal to the desired one (2kHz).

 For the sake of completeness and validation, a similar tuning process was carried out in the Z-domain by finding the discrete time equivalents of the continuous functions presented earlier. The goal was to compare the continuous and discrete time systems. The transformation form continuous to discrete time was carried out with zero order hold (ZOH) method: 𝐺 ∗ (𝑧) =  𝑧 − 1 𝐺(𝑠) 𝒵[ ] 𝑧 𝑠  (4.9)  Having done the tuning process in the Z-domain, equivalent discrete time gains were found: 𝑅𝑇  1 − 𝑒− 𝐿 𝑘𝑖 = 𝑇 𝑘𝑝 =  (4.10)  𝑅𝜔𝑐  (4.11)  𝑅𝑇  1 − 𝑒− 𝐿  𝑟𝑎𝑑𝑖𝑎𝑛𝑠  By substituting the motor parameters, corner frequency ( 𝑠𝑎𝑚𝑝𝑙𝑒 ) and sampling time (25μs), the following gains were found: 𝑘𝑝 = 0.520, 𝑘𝑖 = 2774.

 Table 4-1 Discrete and continuous gains comparison Continuous  Discrete (T = 25μs)  𝑘𝑝  0.503  0.520  𝑘𝑖  2875  2774  The discrete time system was examined in a similar way as the continuous one and the results were compared.

  41 43:5470489815    Figure 4-8 Step response of the discrete time system The discrete step response (Figure 4-8) is very similar to the continuous time one. A slight difference can be seen in the settle time, however not a significant one.

  Figure 4-9 Discrete time open loop Bode plot On the other hand, the discrete Bode plot is quite different from the previous continuous time one. There is a clearly visible phase drop that is increasing with the frequency approaching the sampling frequency. This is a natural feature of discrete time systems, due to nonzero 42 44:1083496869   sampling period. The discrete system has roughly 80° of phase margin, so eventually it may become unstable, unlike the ideal continuous time counterpart. The gain margin is equal to 16.8 dB.

 4.1.2  Simulation results  The GECKOCircuits simulation was created in order to validate the field oriented control implementation. First, it was essential to validate if the closed loop current control system is performing as expected. The PI controller gains were set according to the continuous time system (second column of Table 4-1) and the motor parameters were set according to the QM5006 motor measured values. The virtual motor was stalled by setting a high friction coefficient and a current step was commanded in the q axis. The resulting response was recorded (Figure 4-10)  Figure 4-10 Simulated current step response The recorded response is almost identical with the responses from Figure 4-6 and Figure 4-8.

 The settle time is again about 500us. Having checked the current controllers implementation, it was possible to validate other factors.

 Figure 4-11 presents the motor line-to-line voltages and phase currents. The SVPWM modulation is clearly visible in the voltage waveforms, whereas the currents are purely sinusoidal. To actually observe the currents, the motor had to be loaded with an external 43 45:2144537657   constant torque. In this case, 0.5 Nm was applied and the motor was rotated in the direction counteracting the external torque.

  Figure 4-11 Line to line motor voltages and the corresponding phase currents The next step was to determine the transient behaviour when the motor was rapidly changing the direction of rotation. This experiment was divided into two simulation rounds – with and without d/q axis decoupling.

  Figure 4-12 Currents in d and q axis when sudden change of rotation direction occurs. The decoupling effect is indicated by the red circle.

 44 46:8305417276   There are four quantities visible in Figure 4-12: d set/q set which, correspond to the commanded currents in d and q axis, respectively, and d read/q read which correspond to the measured currents. It is crucial to note that the motor is spinning freely, i.e., it is not loaded with external torque. This is why the q axis current is not reaching the setpoint current value as it is constrained by the maximum motor rotational velocity and cannot accelerate further to keep the setpoint current. The test cycle is started by commanding a positive q axis current. The motor accelerates to its maximum speed, the measured q axis current decreases and after about 80 ms the q axis commanded current is reversed. This causes the motor to deaccelerate rapidly and accelerate in the opposite direction. It is this exact moment, when the d axis current is affected by the coupling term −𝜔𝐿𝑞 𝐼𝑞 from equation (3.6).

 In the coupled case (upper part of Figure 4-12) the d axis current is explicitly affected by the q current change. However, with decoupling implemented by introducing feedforward terms, the axes should become independent. This can be observed on the lower part of Figure 4-12. The influence of q axis current on the d axis was mitigated. The decoupling is especially crucial when dealing with high inductance motors. In this case it could be omitted, however, for the sake of completeness and universality the controller is able to perform d/q axis decoupling based on measured inductance values.

 The simulated waveforms served as a reference for further software implementation on real hardware and allowed for finding possible flaws and controller misbehaviours.

  4.2 Mechanical 4.2.1  Motor selection  The assumption was to use only low cost hobby brushless motors. To make the module as packed as possible, it was essential to pick a motor with a low profile and the largest possible stator diameter, below 45 mm. The last restriction was dictated by the size of the gearbox and its load capacity. Initially, three motors were chosen - Sunnysky X4108s [I14], QM5006 [I15], and Turnigy 4822 [I16]. Then the motors were compared based on the available parameters.

 The KV parameter and phase resistance were used to estimate the motor constant, which is the best metric to compare motors with each other. Equation (4.12) is the formula that was used for torque constant derivation: 𝐾𝑡 =  3 1 60 1 ∙ ∙ ∙ 2 √3 2𝜋 𝐾𝑉 45  47:1051837796  (4.12)   3  where 2 factor indicates a sinusoidal motor is considered, line and phase quantities and  60 2𝜋  1 √3  is for transforming between line to  is used for converting rpm to rad/s. When phase resistance is  known, it is possible to calculate the motor constant: 𝐾𝑚 =  𝐾𝑡  (4.13)  √𝑅  Both read and estimated parameters are listed in Table 4-2.

 Table 4-2 Motor comparison  Parameter  Sunnysky X4108s  QM5006  Turningy 4822  Outer diameter (mm)  46.1  47.5  47.5  Mass (g)  113  87  100  Height (mm)  25.5  20  25  KV rating  380  350  690  Pole pairs  12  14  11  Torque constant (Nm/A)  0.0217  0.0236  0.0119  Motor constant (Nm/√W)  0.0440  0.0492  0.0371  Motor constant per 1 gram of motor mass  0.00039  0.00057  0.00037  Phase resistance ()  0.122  0.117  0.052  Price (USD $)  24.7  25.0  32.3  Presenting a superior motor constant, lower price, and lowest profile height among other motors, the QM5006 motor was chosen. After purchasing, the motor was tested if the estimated parameters match the real measured values. The motor was spun with an electric drill and the resultant line-to-line bEMF voltage signal was recorded. The KV parameter was estimated from the frequency and peak-to-peak voltage. Then the phase resistance was obtained by measuring the voltage drop across the two phases with a known current flowing through them. The measured parameters are listed in Table 4-3.

 46 48:6930558800   Table 4-3 Comparison between specification sheet and measured motor parameters Parameter  Specification (estimated)  Measured  Outer diameter (mm)  47.5  47.6  Mass (g)  87  92  Height (mm)  20  20.1  KV rating  350  293  Torque constant (Nm/A)  0.0236  0.028  Motor constant (Nm/√W)  0.0492  0.056  Motor constant per 1 gram of motor mas  0.00057  0.00037  Phase resistance ()  0.117  0.1153  There is a slight difference in the measured and estimated values of torque constant and motor constant. This might be the result of different magnet strength or inaccurate data posted in the motor specification. Additionally, the shape of the induced bEMF voltage signal was found to be sinusoidal, which indicates the motor is PMSM type. This was a desired feature as it works in favor when FOC control algorithm is used.

 4.2.2  3D model  The main assumption was to create a lightweight actuator, robust to possible impacts and resistant to higher temperatures generated by the loaded brushless motor. Aluminum was chosen as the main structural material, as it is relatively lightweight, is not susceptible to elevated temperatures, and can be milled easily. It is also cheap, easily accessible and can be purchased in a convenient form for milling. The actuator was designed in SolidWorks environment. Existing parts such as motor and the gearbox were measured and modelled, whereas the others were made from scratch in the SolidWorks program.

  47 49:6772940975    Figure 4-13 Exploded view of the actuator The gearbox consists of the internal gear, planet gears with pins, a central sun gear, two aluminum parts serving as planet carriers, two thin-wall bearings, and the outer shell (Figure 4-14). The 4.5:1 gear ratio gearbox is designed in such a way it can be removed as a whole and replaced. This is especially useful when any type of failure occurs. The internal gear has four tabs that are used for locking the gear in the gearbox shell part. Two bearings are press-fitted into the aluminum gearbox housing. Planet gears are rotating on three stainless steel pins, which are pressed fitted into the lower planet carrier. The top carrier’s holes are slightly larger so that it can be removed without pulling out the pins from the lower carrier. A small radial ball bearing is installed in the top carrier to support the motor shaft. This way the shaft is supported on both ends – near the stator and the output of the gearbox.

  Figure 4-14 Gearbox section in detail  48 50:9273888786   The central gear is press fitted on a specially designed shaft extension that is connected to the motor. This part is crucial, as the sun gear may slip under heavy loads if not mounted properly. The shaft is about 3.15mm in diameter and the contact height is 4.5mm, which corresponds to the contact area of roughly 44.5mm2. The peak torque that might be present on this shaft is 0.8Nm. This gives a shearing stress strength required to avoid slippage, of about 28Mpa. Even high strength compounds such as Loctite 620 (25Mpa after curation) would not be sufficient to fix the two parts together. The solution was to use a connection similar to a keyway connection. The shaft was milled with two tabs (keys), whereas the gear was milled with corresponding matching grooves. This way the tabs push against the grooves and form a strong connection. To prevent the weakening of the sun gear, the grooves were not milled all the way through, but only to half of its height.

 Below the gearbox there is the motor section (Figure 4-15). The motor’s stator is pressed onto the milled base part, whereas the rotor’s bearing is pressed into it. The original motor’s rotor is milled with four slots and the top surface is face-milled. The sun gear mount is screwed to the original rotor’s threaded holes. The back side of the rotor shaft is also milled in order to make room for the encoder magnet. A diametrically magnetized magnet with a 2mm hole is used. The rotor shaft has a hole as well. The magnet is placed on the shaft and centered on a 2mm pin placed in the hole with additional adhesive. The rotor can be removed even after fixing the magnet, as is the same diameter as the rotor shaft. Three stator wires are threaded through a slot and connected to the controller board.

  Figure 4-15 Motor section in detail 49 51:8378225453   The last section of the actuator is the controller section (Figure 4-16). The motor controller PCB is placed there and is covered by an aluminum cap. Four threaded holes made in the motor base are used to fix the controller PCB to the base with M2 screws. The PCB is placed in the center in order to align the encoder IC and the rotating magnet. The aluminum cap is used as a heatsink for the MOSFET transistors and the MOSFET driver, as well as for the motor.

  Figure 4-16 Controller section in detail All parts are connected with sixteen M3 screws, screwed to the distancing part. Outer shell parts are not equipped with any centering features, so there is a possibility of slight misalignment. These improvements could be added at a cost of a more complex machining process and thus increased cost. Overall, the module is very compact, which can be noticed in the section view depicted in Figure 4-17.

  50 52:9579492667    Figure 4-17 Section view of the actuator Due to the specific application the actuators are going to be used in, there are additional output mounts used for rotating a special motor harness (hip motor), connecting two motors front to back (thigh motor) and driving a toothed belt (tibia motor) (Figure 4-18). These additional parts allow for a very compact design of a single leg actuation module presented in the last chapter.

  Figure 4-18 Different output mounts. a) mount for rotating a special actuator harness b) mount for rotating another actuator c) mount for driving a toothed belt Some parts were optimized to reduce the total actuator mass (Figure 4-19). The assumption was to keep a certain shell thickness and remove the rest of the redundant material.

 This way, the module’s total mass was reduced by about 20 g.

  51 53:7664552217    Figure 4-19 Parts milled with additional slots to make them lightweight 4.2.3  Manufacturing the actuator  Before the actuator parts were milled, it was printed on a FDM 3D printer to make sure the designed parts were matching the motor and gearbox elements. Moreover, the printed model allowed for finding any possible flaws that could influence the final prototype. The next step in manufacturing the actuator was milling the main parts of the module from aluminum stock. A homemade 3-axis CNC milling machine was used (Figure 4-20).

  Figure 4-20 Homemade CNC machine 52 54:7564973745   The machine runs on a LinuxCNC operating system and has a 280x300x150 mm working area. Thanks to a rigid construction, precision linear rails and ball screws, it is capable of maintaining high precision milling in aluminum, enough for press-fitting bearings or shafts.

 The G-codes were generated using Fusion 360 and then imported into Axis - LinuxCNC control program running on the machine itself.

 The PA6 2017 T4511 aluminum alloy was selected as it has good mechanical properties and can be milled easily. The material was purchased in cylinder slices of different heights and diameters. Each stock was cut with a 1mm margin on each side for the surface facing purposed, due to poor surface finish after cutting the cylinder with a band-saw. All slices were mounted on the plywood table using M3 screws and planned using a 6 mm carbide flat endmill.

  Figure 4-21 Face-milled cylinder slice After face milling, each slice was mounted on the table of a milling machine and further operations were conducted. To save time spent on tool changes and offset measurements, three instances of a single part were milled in one process using multiple work offsets (G54, G55, G56).

  53 55:6223247929    Figure 4-22 Three instances of the same part milled using multiple work offsets The remaining material was used to mill small parts such as the sun gear mount. Milling grooves inside the sun gear was carried out in a precision vice that held the gear firmly.

 If the part required flipping between proceeding milling stages, the plywood table was milled with a centring shape to keep the part in the centre of the actual reference frame. The most complicated part, requiring flipping, was the connector part between the two actuators (Figure 4-23)  Figure 4-23 The most complicated part of the module, requiring many CNC milling stages as well as flipping the part during the process. See Appendix 4 for a time-lapse CNC milling video The motor was modified to fit into the module. The stator was taken off the aluminum housing, which required pushing off the rotor bearings and milling the housing from the inside to loosen the stator. This process was inevitable, as the stator is delicate and it could not be 54 56:1712685303   pushed off the housing without damage. The rotor was modified by face-milling it’s top surface and milling four slots in order to reduce the weight. The motor shaft was shortened to make room for the encoder magnet.

  Figure 4-24 Rotor before and after milling the slots After milling the individual parts, the module was assembled. First, the stator base was covered with Kapton tape to isolate the windings from the aluminum part in case the enamel was scraped off. The stator pinion and stator itself were covered with Loctite compound and pressed together.

  Figure 4-25 motor shaft with two lobes and the sung gear milled with corresponding grooves Then the main rotor bearing was pressed into the motor base. Afterwards, the encoder magnet was glued to the rotor shaft and centered by pressing a 2 mm steel pin. The controller was screwed to the base, thermal conductive pads were applied on the transistors and the driver, and the cap was placed over it. The whole module was screwed to the distancing part. The gearbox was assembled by placing the internal gear and pressing the bearings into the gearbox aluminum housing. Steel pins were pressed to the holes in the lower planet carrier. The gears were installed, the sun gear was pressed onto the motor shaft extension and then screwed to the 55 57:3516092826   rotor. Finally, the gearbox housing was screwed to the distancing part with eight M3 screws.

 The assembled module is presented in Figure 4-26  Figure 4-26 Fully assembled actuator  4.3 Electrical 4.3.1  Controller electrical requirements  The main purpose of the 3-phase motor controller is to run field oriented control algorithm. To do that, the module has to be equipped with a microcontroller, three half-bridges, a position sensor, and current sensors. Moreover, the module has to be able to communicate with a master controller through a high speed communication bus. In addition, the controller should fit into the motor module case. These requirements impose some restrictions on component sizes and the maximum power the controller will be able to handle.

 Maximum phase current and maximum supply voltage are usually the first parameters a motor controller is described with. At the time when the first prototype of the controller PCB was designed, it was not known what is the saturation current for the chosen motor (QM5006).

 This way, the maximum phase current (limited by the MOSFET transistors) was estimated with a certain margin. The torque constant of QM5006 is equal to 0.028 Nm/A. If we assume the maximum torque on the motor shaft cannot exceed 0.8 Nm, it is possible to estimate the phase current at that torque 0.8 𝑁𝑚 ÷ 0.028  𝑁𝑚 𝐴  ≈ 29 𝐴. This calculation does not take into account  the saturation effects likely to happen at currents this high, so the real torque is expected to be 56 58:3592773427   reduced. Moreover, the resistive power loss (Joule heating) at 29 A and 0.115 Ω phase resistance is going to be roughly 145 W, leading to rapid motor heating. Having determined the maximum allowed phase current, it is essential to know the maximum supply voltage as it influences the MOSFET transistors choice as well as power supply distribution components.

 The maximum supply voltage was chosen to be 24 V, however a certain safety margin was also considered.

 Knowing the maximum phase current and supply voltage is a good starting point for choosing appropriate MOSFET transistors. Small area on the PCB dedicated for the transistors narrowed down the available package options to only a few. Eventually, a Vishay PowerPAK 1212-8 was chosen. This package is superior as the manufacturer produces a variety of transistors with different parameters in the same case. In case of any modifications regarding the maximum current or voltage, it is easier to change only the transistors without influencing the PCB layout. Finally, the SISA88DN was selected. The transistor features a maximum drainsource voltage of 30V and a maximum continuous drain current (ID) of 40.5 A. Moreover, it has a relatively low total gate charge (Qg) – 8.3 nC, which directly influences the time it takes for the MOSFET to go into the full conduction state. The channel resistance in the on state is low (Rdson = 0.0067 Ω), resulting in low resistance loss when the transistor is conducting. The dimensions of a single transistor are only 3.3x3.3x1.0 mm.

 In order to switch the transistors very fast, it is essential to use a dedicated driver IC. A Texas Instruments DRV8323RS “smart” driver dedicated for brushless motor control was chosen. It is described “smart” as it supervises the PWM signals being applied on the half bridges and does not allow for a shoot-through, automatically adjusts dead-time and detects any MOSFET faults immediately stopping the commutation process. The driver allows for changing many drive parameters such as gate charge current, additional dead-time, PWM modes, and many more. Moreover, it is equipped with a buck regulator that can be used for powering the microcontroller from the main DC bus. Three built-in operational amplifiers are dedicated to reading the phase currents from shunt resistors, so that no external components are required.

 To implement FOC control, it is necessary to obtain the information about the rotor shaft position. An AMS AS5047B 14-bit magnetic encoder was used to read the rotor angular position. The IC works with a diametrically magnetized magnet located on the shaft. It can operate simultaneously in both absolute and incremental mode. High speed SPI bus is used to read absolute position data, whereas the incremental data is obtained through ABI output.

 57 59:1112899519   The microcontroller is a STM32 G4 series microcontroller, optimized for usage in motor control applications. It features hardware support for fast trigonometric calculations and filtering signals. Built-in timers with encoder support allow for easy integration with external position sensors and hardware ABI signals handling. Moreover, the microcontroller features a CANFD bus peripheral used for high speed data exchange. In order to measure phase currents the double ADC unit was utilized.

 DC bus capacitors are another crucial part of the controller. They supply short-period high current spikes when the transistors are switching, lowering the DC bus voltage ripple. To preserve the low profile of the controller, ceramic capacitors were used. In comparison to electrolytic and tantalum capacitors, they feature lower equivalent series resistance (ESR), and smaller packages. Although they are prone to DC bias derating, which is inherently linked to ceramic capacitors and their internal structure. It means the higher the DC voltage applied to the capacitor, the lower the effective capacitance is. Based on the information stated in [20], it is beneficial to use X7R or X5R dielectric type over Y5V. The C0G dielectric capacitors capacitance is not affected by the DC bias, however, their price excludes them from this application. Eventually, eight 10uF X7R capacitors were chosen, which correspond to at least 40uF at 24V.

 The CANFD peripheral requires an external transceiver used for translating ground referenced microcontroller signals to differential CANFD bus signals. MCP2542 was used as it is CANFD compliant, with a maximum speed of 8Mbps and a small 3x3mm DFN case.

 4.3.2  Schematic and PCB design  After selecting the main components, the schematic and PCB board were designed in Altium Designer 18 program. Recommended designs of each IC were used as a reference when creating the schematic. The main structure was also inspired by other brushless motor control projects [21, 8, 22, 16]. The schematic can be found in Appendix 1.

 Figure 4-27 Depicts the power supply distribution arrangement.

  Figure 4-27 Controller power supply structure 58 60:3605042461   The module power supply can vary from 12 to 24 V. It would not be optimal to use a single LDO to power the 3.3 V devices, as they may consume up to 50 mA and cause excessive LDO heating. To solve that, an internal DR8323RS buck converter was used. The converter outputs 5 V which are forwarded to the LDO. This way the voltage drop on the LDO is significantly lower and thus the heating is reduced. The 5 V bus is also used to power CANFD transceiver so that the LDO is not loaded with additional current.

 The PCB is a small (33x33 mm), low profile, 4-layer design. It has four mounting holes, two connectors, and soldering pads for power supply and the motor. To improve the immunity to interference, some requirements were imposed on the PCB design [23]:   Low inductance paths for charging and discharging the MOSFET gate capacitor. If high transistor switching frequency is desired, it is essential to keep the copper trace loops as small as possible. This improves the current rise time, as well as mitigates the interference caused by induction. The poor and correct layout is compared in Figure 4-28.

  Figure 4-28 a) poor layout resulting in large loops, b) correct layout - using other layers to minimize the area created by the traces. The yellow colour indicates gate charge traces whereas green is used for depicting the return paths  Separate digital and power ground planes. The ground plane is divided into two areas – one for the digital domain and the other for carrying high currents flowing to the motor.

 This way, the high currents and voltage drops associated with them will not influence the sensitive digital components. The two planes are connected in a single point near the DRV8323 driver. Ideally, the connection point should be located in the spot, where 59 61:8916380489   the power supply is connected, however, it was not optimal in this case, due to the location of the MOSFET driver.

  Figure 4-29 Ground plane separation. The digital domain is less affected by the voltage drops and interference caused by high currents in the power domain  Four layer PCB design allowed for creating two internal planes: VCC and GND. This way, the decoupling capacitors can be placed near the ICs, ensuring low inductance current paths. As a good design principle, the ground plane was placed directly under the components layer for improved EMI characteristics. Moreover, the VCC plane was offset with respect to the GND in order to reduce the radiated emission at PCB edges.

 The board layer stack can be seen in Figure 4-30.

  60 62:9947615895    Figure 4-30 Board layer stack  To measure very small voltage drops across each shunt resistor, the traces had to be designed very carefully. Each shunt resistor is connected between low-side MOSFET transistor and the power ground plane. To prevent possible measurement errors due to voltage drop (caused by additional resistance of the terminals), it was decided to use the four-terminal sensing connection (Kelvin contact). The sensing traces were placed close to each other in order to improve the immunity to interference.

  Figure 4-31 Four-terminal shunt resistor connection  61 63:2720913618    Double encoder footprint – in case the main encoder (AS5147) is not available on the market, or the price is too high, another encoder can be used (MA702) without the need of changing the whole PCB layout. This was accomplished by placing two footprints in the same PCB region, so that the two ICs can be used interchangeably.

 The resulting PCB design, divided into four individual layers, is shown in Figure 4-32.

  Figure 4-32 Individual layers of the motor controller PCB To be able to solder many controllers at once, a solder stencil was purchased with the PCB boards. The stencil was used to distribute the soldering paste on the exposed pads. Then the components were placed on the PCB board and soldered using reflow technique in a small oven. Reflow technique allows for repeatable solder joints and superior visual effects, even without cleaning the board after soldering. A completed PCB after soldering is shown in Figure 4-33.

  62 64:1047940865    Figure 4-33 Soldered motor controller, mounted on the aluminium base part  4.4 Software 4.4.1  Commutation routine  To achieve high bandwidth current control and minimize voltage ripple due to PWM switching on a relatively low inductance motor, a high PWM frequency had to be chosen (40 kHz). This high PWM frequency results in less than 25μs for all measurements and calculations required by the FOC algorithm.

  Figure 4-34 Actions performed in the 25 μs window Due to strict time constraints, it was necessary to use 32-bit float type for calculations, which can be computed on the hardware FPU unit with low latency. At the same time, any casting actions to 64-bit double type had to be omitted as it introduced a few microseconds of additional loop time, exceeding the 25 μs boundary.

 In order to keep the tight time constraints, the interrupt handler had to be optimized.

 The encoder position SPI readout function from STMicroelectronics HAL library was too slow.

 It was replaced with a direct register operating function that was fast enough to fit in the fewmicrosecond time window. Another relatively slow operation was the sine and cosine function calculation, needed for the FOC algorithm transformations. There were three available options 63 65:7521924261   that were considered – sinf() and cosf() from the math.h library, lookup tables or CORDIC peripheral. The first solution was to use the build-in float type functions which were expected to be the slowest, although the easiest to use. The lookup table solution has the biggest memory usage amongst other solutions, however it should be much faster than the built-in sinf/cosf functions. Coordinate Rotation Digital Computer (CORDIC) is an internal co-processor of the STM32 G4 series microcontroller. It is used for accelerating trigonometric calculations for motor control and signal processing applications, saving the main processor time.

 A test was carried out to determine the fastest implementation. All three solutions were used to calculate sin(x) and cos(x) functions for x varying from 0 to 14π radians. A wider argument range than the sin/cos base period was chosen in order to detect any execution time increase due to modulo division that is implemented to cut down the angle range to 0-2π radians.

 The results are depicted in Figure 4-35.

  Figure 4-35 Execution time comparison between different sin(x) and cos(x) calculation algorithms Presumably, the math.h library trigonometric implementations were the slowest, and highly dependent on the angle, especially in the 0-π range. The lookup table solution is much faster, especially at angles greater than π radians, however, the greater the angle above the base sin(x) period, the more time it takes to calculate it. This could be notably observed with high pole number motors. The CORDIC implementation is undoubtedly the fastest, as it takes less than a microsecond to complete both sin(x) and cos(x) calculations, and the execution time is 64 66:1000638120   not angle-dependent. The accuracy of each implementation was determined, by comparing it to the double-type math.h function. It was found that the fsin() and fcos() functions were the most accurate with a maximum percentage error of 8.74e-6 %, the CORDIC implementation featured 0.018% of maximum percentage error, and the least accurate was the table implementation that gave 0.614% of maximum percentage error. For the hardware setup in the presented actuator the CORDIC implementation is the most sufficient one, considering both accuracy and execution time.

 Thanks to these optimizations, the single algorithm loop pass takes around 20-21 μs.

 4.4.2  Timer  The hardware Timer1 is used for generating the motor PWM signals. It works in the center-aligned mode, which means it counts up and down and the PWM signal is aligned with the period center. The interrupt is being executed only when the timer counter reaches the autoreload value (2249). This ensures that the interrupt is started in the 0th sector when all low side switches are closed (Figure 4-36).

  Figure 4-36 Timer to PWM operation on each phase. Note – the PWM waveforms are in regard to high side switches – 1 means closed, 0 means open. The low side switches are complementary.

 65 67:9268555566   4.4.3  ADC measurements  The algorithm presented in Figure 4-34 starts with ADC current measurements used for computing d/q axis currents. To get accurate measurements, the currents have to be sampled in the middle of the 0th sector (at the very beginning of the interrupt handler). Only two ADCs are used for sampling currents, as the third one can be calculated based on the first Kirchoff’s law.

 Obtained voltage drop measurements (across each shunt resistor) are converted to currents in amperes. Later on, they are used to calculate the d/q axis currents.

 The motor temperature and DC bus voltage are sampled right after the phase current measurement is returned by the ADCs. Motor temperature is measured to detect possible overheat events, so that the motor is safely turned off without burning the coils or affecting the magnet strength. The DC bus voltage is monitored for overvoltage events likely to happen when the motor is braking or changing direction rapidly and the energy is injected to the bus through freewheeling MOSFET diodes. This causes a relatively long (in the millisecond range) voltage spike above the nominal DC bus voltage that can damage the hardware. As there is not enough space in the module for an additional breaking resistor and additional transistor, the extra energy is dumped in the d axis by commanding a nonzero d axis reference current. Assuming the actuator is working in a legged robot where the overvoltage events are intermittent, the motor can be used as a breaking resistor without the chance of overheating.

 This technique could not be sufficient for continuously rotating motors in applications such as e-bikes or electric skateboards, where the breaking resistor is indispensable. In case of overvoltage detection, the controller commands a nonzero current in the d axis to dump the energy in the motor.

 4.4.4  Position and velocity measurement  The second block depicted in Figure 4-34 symbolizes the rotor position and speed measurements. As mentioned earlier in this chapter, the encoder communicates with the microcontroller through two buses – SPI (for absolute output) and ABI (for incremental output).

 The absolute output is used for determining the magnetic offset and for position calculations, whereas the incremental output is used for velocity measurement. Similarly to [21], the velocity is calculated by measuring the time (t) between two respective encoder timer ticks (Figure 4-37).

  66 68:6947042943    Figure 4-37 Velocity measurement based on time intervals This approach utilizes two timers – one working in encoder mode, that counts up and down, depending on the motor rotation direction, and the second which measures the time intervals between the respective encoder timer ticks. This approach allows for high frequency velocity output, even at low speeds, and works in the background, which is superior to manual position differentiation. The velocity signal is relatively noisy, so a 250 sample moving average filter was used, as it does not influence the data output frequency. The filter was implemented in two manners - using FMAC (Filter Math Accelerator) hardware filtering peripheral and directly in the software. The execution time difference was insignificant, so the author decided to use the software implementation as it allowed for increasing the filter sample count when needed.

 4.4.5  Encoder calibration routine  There are two parameters that have to be calibrated beforehand, and a motor can be run for the first time. The first one is the offset between the zero position of the encoder and the true zero position of the rotor with respect to the magnetic poles (Figure 4-38). The offset is measured by commanding a nonzero voltage in d axis, zero voltage in q axis and setting the theta angle to 0. This causes the rotor to align with the d axis – “zero” position of the motor’s rotor. After the rotor settles, the controller reads the position from the encoder and saves it as an offset. The offset is subtracted every time a position is sampled. This ensures the controller always knows where the zero position of the rotor is and thus is able to generate an appropriate commutation.

 67 69:9876850443    Figure 4-38 Offset definition The other calibration process is to compensate for the eccentricity of the magnet and the encoder. It allows to mitigate the effects of non-axial placement between the encoder and the magnet placed on the rotor’s shaft. Even a small placement error caused by sloppy soldering of the encoder sensor IC can lead to relatively big torque variations within a single rotation.

 This can be accounted for with a calibration process. During the calibration, the controller rotates the rotor slowly by a full rotation and records the error between the setpoint angle value and the read value from the encoder. Afterwards, the controller filters the position samples to get rid of cogging effects or vibrations and constructs a 128-point lookup table. The table is saved in flash memory so that the calibration has to be done only once per motor module.

 During the operation, values from the lookup table are subtracted or added to the read position values to compensate for sensor eccentricity. One has to remember that after any mechanical change in position of the sensor and the magnet, the calibration routine should be performed again.

  68 70:6871282762    Figure 4-39 The position error due to magnet-encoder misalignment before and after the calibration routine Although the motor module is equipped with an encoder, it cannot explicitly determine its output position after startup. This is due to the planetary gearbox presence which causes multiple motor shaft rotations per single output shaft rotation. Therefore, after the startup, the output shaft can be in one of the four and a half sectors (due to the gear ratio of 4.5:1) from the motor encoder’s point of view. A solution to this is to put the shaft in the known position before the motor is powered on. As there are four and a half angular areas that can be distinguished, placing the output shaft within an 80° window should be enough for the controller to explicitly determine its output shaft position. Each motor module has to be calibrated beforehand by placing its output shaft in a known position and saving the encoder measurement. This value indicates the position of the actuator output shaft with respect to the motor’s case. To keep the same amount of possible error on each side of the saved position, the encoder reading is offset in such way that it always equals 180° at the saved output position (Figure 4-40). This offset is only valid within one 80° slot.

  69 71:8905872754    Figure 4-40 Encoder position offset – the output shaft reference point must lie within the green region for correct position initiation If the device gets powered on in any other slot (other than that it was calibrated in), it is going to assume the saved position is somewhere else, leading to false output position readings.

 Additional encoder could be used to determine the output shaft position precisely without any calibration routines. This solution, however, is more expensive and requires axial placement on the output shaft, which would greatly complicate the mechanical structure and could lead to additional costs. Some absolute encoders can be powered with a small sustaining battery just to keep the internal position counter. This eliminates the need for offset calibration, but comes with increased volume (sustaining battery), cost, and failure rate.

 4.4.6  Current measurement calibration  To correctly measure the motor phase currents, it is essential to calibrate the current offsets. Without calibration, the controller sees the current waveforms shifted from the zero current level, which influences the amount of torque ripple. To fix that the motor is slowly spun and the current waveforms are analyzed – their maximum and minimum values are used to calculate the offset. After applying the resultant offsets, the current waveforms are centered on the zero current level and the torque ripple is minimized. Figure 4-41 presents the current waveforms before and after the calibration.

  70 72:4585684519    Figure 4-41 Before and after the calibration process. The waveforms look similar to trapezoidal ones as there is no position control during this calibration routine – the rotor simply follows the set-point voltage waveforms (open-loop position control).

 4.4.7  Automatic motor parameters identification  The controller is also able to detect motor internal parameters such as phase resistance and d/q axis inductance. Measuring resistance is based on the steady state current flowing through the motor coils. The controller commands a certain voltage and after a few milliseconds (more than 5 time constants of the LC circuit) it samples the current flowing through the resistive load. The process is repeated 20 times and the measured resistance is averaged. Phase 2  resistance equals to 3 of the measured value (due to the wye phase connection style).

 The inductance measurement is somewhat more complicated. Before the measurement is made, the resistance must be acquired. After that, the rotor is aligned with the d axis and a voltage step is applied in the d axis. A timer is used to count the time until the current response reaches 63.2% of the steady state current. The steady state current is calculated from the step voltage and measured resistance. The q axis inductance measurement works in a similar manner, although the rotor is aligned with the q axis. The biggest issue of this method is that 71 73:1365920637   the current rising in the motor coils causes a torque that rotates the rotor. This is why the q axis inductance measurement may be inaccurate to some extent as even a fixed rotor shakes slightly when the q axis voltage is commanded. However, since only surface mount permanent magnet motors are considered, the difference between d and q axis inductance is not going to be significant.

 4.4.8  High level control  Currently, the actuator is capable of operating in the spring-damper mode (PD control).

 This high-level control is used by many legged robots to ensure safe interaction (compliance) with the environment [8]. The setpoint torque (q axis current) is calculated as: 𝜏 = 𝐾𝑠 (𝛩𝑑𝑒𝑠 − 𝛩𝑎𝑐𝑡 ) − 𝐾𝑑 (𝛩̇𝑎𝑐𝑡 )  (4.14)  where 𝛩𝑑𝑒𝑠 is the target actuator shaft angle, 𝛩𝑎𝑐𝑡 is the actual angle, 𝛩̇𝑎𝑐𝑡 is the actual rotational velocity, and 𝐾𝑠 , 𝐾𝑑 are spring and damping constants, respectively. The interaction can be varied with the spring and damping constants, based on the desired actuator behavior. The results of this type of high level control are presented in the results chapter.

 4.4.9  CANFD communication  The controller area network flexible data-rate (CANFD) is the main channel used for exchanging information with the module. The classical CAN bus was originally created to communicate with vehicle onboard devices and sensors. It is known for its great immunity to interference and hardware error handling. The CAN flexible data rate means the controller is able to send and receive 64 bytes of data at 8Mbps, whereas classic CAN is able to send up to 8 bytes at maximum 1 Mbps. The CANFD bus is used for commanding new position setpoints, changing persistent settings and updating module firmware.

 Communication with a PC requires a converter serving as a translator between USB and CANFD bus. The communication channel must be appropriate for sending messages from and to the master device, so a two-directional data flow is required. This kind of communication structure can be prepared with USB CDC (“Communication Device Class”) library. The microcontroller manufacturer – STMicroelectronics - provides a library which serves as USB to serial converter. After plugging the device to the USB socket, it introduces itself as “Virtual COM Port (VCP)”. This way, the computer is able to forward the serial commands to this virtual COM port, which are received by the microcontroller. There, the message should be rewritten  72 74:9064234570   to the CANFD frame format and sent through CANFD bus to the slave device. Figure 4-42 depicts the structure of the communication system.

  Figure 4-42 Structure scheme of the bootloader system Starting from the left, there is a PC running a Python script that sends data frames to the COM serial port. The converter microcontroller receives USB packages and translates them to serial using the STM32 CDC library. Afterwards, the serial message is rewritten to CANFD frame format and sent to the slave device. The communication is bidirectional.

  Figure 4-43 USB to CANFD converter A special USB to CANFD converter had to be designed from scratch (Figure 4-43). The STM32G431 microcontroller used on each motor module was chosen for this purpose as it is equipped with both CANFD and USB bus. On the PCB there is also a CANFD transceiver, TVS protection diodes for the USB bus, and a small 3.3 V LDO regulator. Three LED diodes are used for interaction – two of them act as status diodes, whereas the third is used to indicate errors. The board is only 35x26 mm. The case for the PCB was designed in SolidWorks program and later milled on the milling machine.

 73 75:3837334437   4.4.10 Bootloader A bootloader is a special kind of firmware, usually very compact, placed in the beginning of the microcontroller’s memory. This location ensures the bootloader is always executed when the device is restarted. The bootloader’s main function is to check if it should download and then program a new firmware, or if it should continue executing the user’s code. Bootloaders are used to omit the need of using an external programmer. Whenever a firmware upgrade is needed, it can be downloaded over popular communication buses such as serial bus, SPI bus, or even wireless protocols. There are two types of bootloaders commonly used in microcontrollers: 1) The bootloader has to be able to receive the binary firmware file through a specific communication channel, check if it’s not corrupted, and load it into the internal flash memory.

 This kind of bootloading process is the safest, ensuring that even if the main code is not functional, the firmware can always be replaced.

 2) The bootloader is only required to copy the firmware from one place in memory to the other. The whole firmware downloading process is performed by the main code. A major difference from the first bootloader type, especially important for small memory chips, is that the bootloader code itself is very small. This is due to the fact that it has to only copy one section of the memory into another location. No communication nor fault checking is required. Major drawback of this solution is when the main code does not work as expected and is unable of downloading its own repaired firmware. This case requires an external programmer.

 Considering the difficult accessibility of each motor controller and its programming connector, when mounted in the leg structure, the most appropriate type of bootloader for actuator application is the first type. At the cost of higher memory footprint, the fault immunity of the whole system is greatly improved.

 The motor controller described in this thesis has a main CANFD communication channel, which is the best way of downloading the firmware. All STM32 microcontrollers are able to replace their firmware through a few different buses with onboard factory programmed bootloaders. However, these bootloaders are placed in ROM memory and cannot be customized or overwritten. This means that the custom system must be adapted to the strict rules from the microcontroller’s documentation, which is not always sufficient. On the other hand, by designing a custom bootloader, one can adapt it for the specific requirements of a particular system without relying on the factory bootloaders. This was the approach chosen in this thesis.

 74 76:6904319337   4.4.11 Updating the firmware The master device communicating with a multislave system must be capable of recognizing each device based on its identification number. CANFD bus peripheral is capable of filtering the messages based on the standard 11-bit (2.0A) or extended 29-bit identification number (2.0B). This means that whenever a device that listens for messages receives a message, which identification number is out of the desired range, the message is going to be ignored on the hardware level. Using this method, the master device can poll the bus for each known ID and wait for the response. If the response is obtained, a slave device is connected to the bus and is ready to be updated. This way, each device can be polled individually, preserving the same command IDs.

 Each device must be aware of its CANFD address at all times, even after a power down.

 This is assured by placing a special block of information in the FLASH memory of the microcontroller. This block is placed in the last sector of the memory, ensuring it will not be overwritten after a firmware update. At startup, the CANFD peripheral of each module is initialized with different address read from FLASH memory.

 The bootloader itself is placed on the beginning of FLASH memory. Considering the special location, it is accessed each time the microcontroller is powered on or any other reset event happens. Reset flags located in CSR register can be used to distinguish between different reset sources. Implementing this selective strategy, the bootloader waits for the firmware update command only after a software reset is detected. If a normal “power on” reset happened, the bootloader will perform an immediate jump to the user’s code. A software reset can be initiated by the device itself, when it receives a reset request through CANFD bus. This way, the bootloader is transparent to the user, but still can be easily accessed by the CANFD commands.

 A small delay of 500 ms could be added in case of power on reset, as it allows for entering the bootloader mode even if the main code is not functional.

 The main firmware code is placed above the bootloader starting from a predefined memory address. After “power on” reset, the bootloader jumps directly to the specified address and the user application execution is initiated. The application is capable of CANFD communication. When it receives a reset command from the Python script running on the master computer, it performs a software reset. The bootloader is entered, the software reset is recognized, and the bootloader waits 5 seconds for a confirmation command. If the command is not acquired, the bootloader jumps back to the user application. However, if the command is 75 77:8782645482   received, the bootloader acknowledges the computer, it is functional and waits for erase and program commands. The erase command is telling the bootloader how much flash memory has to be erased to fit the user code. After erasing the memory, the bootloader receives 32 byte packets of firmware. The data is gathered until a 512 byte buffer is filled. Then a CRC (“Cyclic Redundancy Check”) code is sent by the master computer and the microcontroller calculates its own buffer CRC code. The codes are compared on the slave device and only when they are equal a write to the flash memory is performed. The procedure continues until the end of the binary file on the host computer is reached. At the end, a jump command is executed and the user application is started.

 4.4.12 Control application A simple Python application was created to communicate with individual modules, update their firmware, and check for faults (Figure 4-44). The app was written using TKinter library for window applications in Python.

  Figure 4-44 Service PC application  76 78:4213292712   After startup, the user has to choose the correct COM port and speed (baudrate) the CANFD converter is operating at. When the “Open port” button is clicked, the app scans for any connected devices and lists them in the drop down list. The chosen ID number is the currently considered actuator. The upper part of the application is used for updating the actuator firmware, by choosing the binary file and clicking “Flash device” button. The lower part is used for diagnostics and calibration routines. The actuator can be operated in the spring-damper mode by varying the target position, spring constant, and damping constant. The user has the ability to calibrate the encoder offset (align encoder zero position with the d axis), calibrate for eccentric magnet-encoder placement, measure motor internal parameters (inductance, resistance), and calibrate current measurements offsets. There is an option for displaying errors that may occur during calibration routines or normal operation. The errors can be cleared if desired. It is also possible to modify the initial position from which the actuator starts its operation, as well as the physical boundaries the shaft has to stay in. Below the buttons, there is a graph that can be used to view the actual position, velocity, d/q axis currents, DC bus voltage, and motor temperature. The terminal is located on the very bottom of the app window and is used for displaying calibration results, information, and error messages.

  77 79:6971296164    5 RESULTS 5.1 Initial waveform validation After completing the hardware and software part of the project, the module was tested.

 Initially, the actual voltage/current waveforms were compared with the simulated ones to check if everything works as expected. The motor was spun with a nonzero q axis (2 A) current and loaded with an external constant torque to actually observe the phase currents.

  Figure 5-1 Voltage and current waveforms recorded from the real hardware Waveforms presented in Figure 5-1 are similar to the simulated ones (Figure 4-11).

 There are some distortions that result from not uniform motor load, however the overall shape is correct. The SVPWM modulation is clearly noticeable in the line-to-line voltages, whereas the currents are sinusoids, limited by the setpoint q axis current.

  5.2 Torque transducer test bench To measure the actuator torque capability, a simple torque test stand was built. The torque sensor used on the stand is a dynamic torque transducer with load capacity up to 5 Nm.

 It is mounted in the center of the board and can be used as both a static and dynamic sensor.

 The actuators are equipped with additional 8 mm output shafts. CNC precision couplers are 78 80:3381396473   used for connecting the actuator shafts and torque transducer shaft. The torque sensor outputs a voltage signal that is proportional to the torque acting on the shaft.

  Figure 5-2 Static torque measurement stand Torque capability measurement was the first test performed on the test bench. One shaft end of the torque transducer was fixed to the stand to prevent it from rotating, while the other was coupled to the examined actuator. The motor module was commanded with gradually increasing q axis current, and cooled down to room temperature between the following current steps to minimize the influence of changing motor parameters due to increased temperature.

 The gathered data allowed for creating a graph of the output actuator torque vs q axis current (Figure 5-3).

  Figure 5-3 Torque vs q current characteristic  79 81:7229826693   The torque constant was determined from the linear region of the graph. A linear fit was performed to the 0-22A region, which resulted in a coefficient of determination of 0.997.

 𝜏 = (0.103 ± 0.00088)𝐼𝑞 + (0.110 ± 0.0113) The torque constant is simply the slope of the linear fit (5.1), which is equal to 0.103  (5.1) 𝑁𝑚 𝐴  . The  saturation region started to show up on the graph when the commanded current exceeded 22 A.

 In this region, the motor can operate up to only a few seconds without reaching the temperature limit. Short high current bursts can still be performed. These are especially useful when jumping or moving in a very dynamic manner. Maximum torque of the module was determined to be around 3 Nm at 35 A of phase current.

  5.3 Thermal testing The next experiment was the thermal response of the actuator under different q axis current commands, with its output fixed to the stand. The temperature was measured with an internal thermistor built into the stator. Each test lasted for about 1.5 h, which corresponds to about five time constants of the system. The time constant was found after the initial test when the temperature of the actuator settled. All experiments were performed in 25 °C ambient temperature and the initial temperature of the motor module was 30 °C. The results are presented in Figure 5-4.

  Figure 5-4 Real and estimated motor thermal responses 80 82:4721680738   An attempt was made to analytically determine the transfer function of the system to estimate the time needed for reaching maximum allowed temperature, knowing the applied current. Based on steady state temperature value the thermal resistance was found. The time constant was obtained by measuring the time when the temperature reaches 63.2% of its steady state value. The input to the system is power loss due to Joule heating that was calculated as: 𝑃𝑙𝑜𝑠𝑠 =  3 2 𝐼 𝑅 2  (5.2)  The output is the temperature rise measured by the thermistor. The system was assumed to be first order. After implementing the temperature estimation based on this model, it was found that the thermal resistance to ambient varies significantly depending on the applied current. The most probable cause was the hot air convection phenomenon, which caused the nonlinearity. The model cannot accurately estimate the system dynamics, however it works well enough in the vicinity of the real thermal response.

 The thermal characteristics were used to determine the maximum continuous torque, which is the amount of torque that the motor is able to generate infinitely without reaching its thermal limit. The continuous torque of this module was determined to be 0.875 Nm. The module is able to operate at higher output torques, however, the active time is limited – 3 Nm can be generated for maximum 7 s before reaching the thermal limit (60°C).

  5.4 Parameter identification and decoupling Besides the torque test bench tests, other experiments were also performed. As mentioned earlier, the module is able to identify the motor parameters such as phase resistance and inductance in the d and q axes. To validate the automatic inductance measurements, a RLC meter was used, which features 0.5% of the base accuracy. For measuring the phase resistance, the method of pushing a constant current was implemented and the respective voltage drop across the windings was recorded. Each measurement, both automatic and manual, was repeated 10 times for increased accuracy. Table 5-1 presents the experiment outcome.

 Table 5-1 Manual vs automatic motor parameters identification Measured quantity Phase resistance [m] D axis inductance [µH] Q axis inductance [µH]  Manual measurement 122.8±9.26e-4 34.4±1.02e-6 48.9±1.58e-6 81  83:1100382350  Automatic measurement 118.2±2.54e-4 32.9±1.41e-6 40.8±6.72e-5   The resistance identification feature works correctly, however, the motor has to be kept at a certain temperature for the results to be reproducible. The inductance, on the other hand, varies by roughly a few μH in regard to the RLC meter measurement. This is especially visible in the q axis inductance measurement and most probably it is caused by the rotor being unaligned with the q axis. In order to perform a high accuracy measurement, the rotor should be perfectly fixed with respect to the stator, as the q axis current generates torque and causes the rotor to rotate if not fixed. Even though, the range of measurement is correct, when compared to the RLC meter measurements. Having implemented the basic parameter identification, it is possible to implement an automatic PI controller tuning routine based on the identified parameters.

 The measured motor inductance was also used for decoupling technique and comparison with the simulated results in Figure 4-12. Measured waveforms are presented in Figure 5-5.

  Figure 5-5 Validation of the d/q axes decoupling technique on the real controller Presumably, the influence of q axis current change on the d axis current after implementing the decoupling technique is reduced. Although, similarly to the simulated results, 82 84:3909937350   the improvement is not crucial to the controller operation and would be more visible on a higher inductance motor.

  5.5 High level control The actuator’s controller is able to perform high level impedance control according to (4.14) formula. In order to validate the implemented control algorithm, a test was carried out in which the spring and damper constants were varied. It was expected to observe a similar behavior as a spring-damper-mass system, i.e.: undamped sinusoidal oscillation with 𝑘𝑠 > 0 and 𝑘𝑑 = 0, exponentially decaying sinusoidal oscillation with 𝑘𝑠 > 0 and 𝑘𝑑_𝑐𝑟𝑖𝑡𝑖𝑐𝑎𝑙 > 𝑘𝑑 > 0, and critically damped response with 𝑘𝑠 > 0 and 𝑘𝑑 ≥ 𝑘𝑑_𝑐𝑟𝑖𝑡𝑖𝑐𝑎𝑙 .

  Figure 5-6 Undamped sinusoidal response During each experiment, the motor module was fixed to the test bench and it was commanded a position step from 0 to 2 radians. The 𝑘𝑠 was set to a constant value of 2  𝑁𝑚 𝑟𝑎𝑑  ,  whereas the 𝑘𝑑 was varied between each run. A low value of 𝑘𝑠 allowed for low frequency oscillations that are easier to capture, however it also contributes to a higher steady state error that can be observed in the following figures. Figure 5-6 presents the case when 𝑘𝑑 is equal to 0. Supposedly, the response is unstable, resulting in continuous sinusoidal oscillations.

 83 85:8159613250    Figure 5-7 Damped sinusoidal response The next experiment run (Figure 5-7) was obtained by commanding a 𝑘𝑑 = 0.01  𝑁𝑚∙𝑠 𝑟𝑎𝑑  . The  resultant response is an exponentially decaying sinusoid. A certain amount of steady state error can be noticed.

  Figure 5-8 Critically damped response 84 86:4846046331   The last response is critically damped, which results in no oscillations (Figure 5-8). At the same time, the steady state error is much larger than in previous scenarios. This can be mitigated in some limited range by increasing 𝑘𝑠 , however oscillations may be introduced. A video of actuator operation in the spring damper mode can be found in Appendix 3.

 The impedance control is a useful technique, when a certain level of compliance with the environment is expected. Moreover, it allows for better impact mitigation in comparison to position-controlled actuators.

  5.6 Summary and future work The module met the functionality requirements stated at the beginning of this thesis. It is easily backdrivable, due to low gear ratio gearbox and thus a certain level of compliance can be achieved. High precision torque output, which is directly proportional to the q axis current allows for commanding as well as reading torques. The module features high maximum torque considering its mass and volume. Moreover, high torque bandwidth can be achieved thanks to a high frequency execution of the FOC algorithm and no compliant elements used in the actuators drivetrain. The aluminum case forms a strong structural shell, protecting the internal components and is used as a heatsink. The actuator was repeatedly tested and no mechanical failures were observed. It is rigid and resistant to shock. Main actuator parameters are listed in Table 5-2.

 Table 5-2 Actuator parameters Dimensions  36 mm x 59 mm  Mass  210 g  Maximum peak torque  3 Nm  Maximum continuous torque  0.875 Nm  Maximum speed (15V)  30 rad/s  Torque control bandwidth  2 kHz  Gear ratio  4.5:1  Supply voltage  10-24 V  Maximum phase current  35A  85 87:4007903377   At the moment, the most problematic aspect is the backlash that excludes the module from applications such as robotic manipulators or haptic devices. The application in the walking robot’s legs is not that demanding, in regard to positioning accuracy, so the module can be successfully implemented there. The gearbox can be easily replaced if needed, without influencing the rest of the module. To further validate the actuator robustness, a prototype of a single quadruped robot led is going to be built. The leg is going to be mounted on a vertical stand with linear rails and programmed to perform repetitive jumps. This is the most demanding test as it simulates the harsh conditions likely to occur in a walking robot.

 In the author’s opinion, the module achieved the low-cost goal. It utilizes easily accessible materials, hobby grade brushless motor, and relatively simple manufacturing processes, achieving satisfactory results. The material cost, per single module is presented in Table 5-3.

 Table 5-3 Estimated material cost of a single module Name  Estimated cost [PLN]  1  PCB components  140  2  4-layer PCB  20  3  Aluminum stock  25  4  Electric drill gearbox  20  5  Bearings  35  6  QM5006 motor  90  7  Pins, screws, cables  25  Sum  355  Although the material cost is low, the amount of time and effort spent on the CNC machine has to be taken into consideration. This is why it is hard to estimate the real cost of a single actuator. Even though, to the author’s knowledge, there are no commercial brushless based low gear ratio actuators with similar form factor.

 The future work includes building single, three-degrees-of-freedom leg (Figure 5-9) and testing it on a vertical linear rail stand. Eventually, the actuators are going to be used in a small 86 88:7428383977   size, four-legged robot, when fully tested and proven to be reliable. Figure 5-9 presents the 3d model of a single robot limb.

  Figure 5-9 Future 3DoF quadruped robot leg render  87 89:8323416263    LITERATURE [1]  B. Katz, J. D. Carlo, and S. Kim, “Mini Cheetah: A Platform for Pushing the Limits of  Dynamic Quadruped Control,” in 2019 International Conference on Robotics and Automation (ICRA), pp. 6295–6301, 2019.

 [2]  P. M. Wensing, A. Wang, S. Seok, D. Otten, J. Lang, and S. Kim, “Proprioceptive  Actuator Design in the MIT Cheetah: Impact Mitigation and High-Bandwidth Physical Interaction for Dynamic Legged Robots,” IEEE Transactions on Robotics, vol. 33, no. 3, pp. 509–522, 2017.

 [3]  P. Billeschou, N. N. Bijma, L. B. Larsen, S. N. Gorb, J. C. Larsen, and P. Manoonpong,  “Framework for Developing Bio-Inspired Morphologies for Walking Robots,” Applied Sciences, vol. 10, no. 19, 2020.

 [4]  S. H. Collins, M. Wisse, and A. Ruina, “A three-dimensional passive-dynamic walking  robot with two legs and knees,” The International Journal of Robotics Research, vol. 20, no. 7, pp. 607–615, 2001.

 [5]  M. H. Raibert, J. H. Benjamin Brown, and M. Chepponis, “Experiments in Balance  with a 3D One-Legged Hopping Machine,” The International Journal of Robotics Research, vol. 3, no. 2, pp. 75–92, 1984.

 [6]  M. Hutter, C. Gehring, D. Jud, A. Lauber, C. D. Bellicoso, V. Tsounis, J. Hwangbo,  K. Bodie,  P. Fankhauser,  M. Bloesch,  R. Diethelm,  S. Bachmann,  A. Melzer,  and  M. Hoepflinger, “ANYmal - a highly mobile and dynamic quadrupedal robot,” pp. 38–44, 2016.

 [7]  N. Kau, A. Schultz, N. Ferrante, and P. Slade, “Stanford Doggo: An Open-Source,  Quasi-Direct-Drive Quadruped,” in 2019 International Conference on Robotics and Automation (ICRA), pp. 6309–6315, 2019.

 [8]  B. G. Katz, “A Low Cost Modular Actuator for Dynamic Robots,” Master’s thesis,  MASSACHUSETTS INSTITUTE OF TECHNOLOGY, June 2018.

 [9]  D. C. Hanselman, Brushless Permanent Magnet Motor Design. The Writers’  Collective, 2003.

 [10]  J. R. Mevey, “Sensorless Field Oriented Control of Brushless Permanent Magnet  Synchronous Motors,” Master’s thesis, Kansas State University, 2009.

 88 90:7920950142   [11]  S. Derammelaere, M. Haemers, J. De Viaene, F. Verbelen, and K. Stockman, “A  quantitative comparison between BLDC, PMSM, brushed DC and stepping motor technologies,” in 2016 19th International Conference on Electrical Machines and Systems (ICEMS), pp. 1–5, 2016.

 [12]  T. Liu, Y. Tan, G. Wu, and S. Wang, “Simulation of PMSM Vector Control System  Based on Matlab/Simulink,” in 2009 International Conference on Measuring Technology and Mechatronics Automation, vol. 2, pp. 343–346, 2009.

 [13]  M. Marufuzzaman, M. B. I. Reaz, M. S. Rahman, and M. A. M. Ali, “Hardware  prototyping of an intelligent current dq PI controller for FOC PMSM drive,” in International Conference on Electrical Computer Engineering (ICECE 2010), pp. 86–88, 2010.

 [14]  A. Zentai and T. Daboczi, “Improving Motor Current Control Using Decoupling  Technique,” vol. 1, pp. 354 – 357, 02 2005.

 [15]  M. N. Gujjar and P. Kumar, “Comparative analysis of field oriented control of BLDC  motor using SPWM and SVPWM techniques,” pp. 924–929, 2017.

 [16]  P. Fisher, “High Performance Brushless DC Motor Control,” Master’s thesis,  CQUniversity Australia, May 2014.

 [17]  A. Musing and J. W. Kolar, “Successful online education - GeckoCIRCUITS as open-  source simulation platform,” pp. 821–828, 2014.

 [18]  TexasInstruments, InstaSPIN-FOC and InstaSPIN-MOTION User’s Guide, 2013.

  [19]  L. Dixon, “Control Loop Cookbook,” 2001.

  [20]  M. D. Waugh, “Design solutions for DC bias in multilayer ceramic capacitors,”  Electronic Engineering Times, pp. 34–36, 2010.

 [21]  B. G. Katz, “Low Cost, High Performance Actuators for Dynamic Robots,” Master’s  thesis, MASSACHUSETTS INSTITUTE OF TECHNOLOGY, June 2016.

 [22]  S. Kalouche, “Design for 3D Agility and Virtual Compliance using Proprioceptive  Force Control in Dynamic Legged Robots,” Master’s thesis, Carnegie Mellon University, Pittsburgh, PA, August 2016.

 [23]  H. Johnson and M. Graham, High-speed Digital Design: A Handbook of Black Magic.

  Prentice Hall Modern Semiconductor Design, Prentice Hall, 1993.

