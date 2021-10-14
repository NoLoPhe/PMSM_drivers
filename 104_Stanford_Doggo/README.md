Link: https://arxiv.org/ftp/arxiv/papers/1905/1905.04254.pdf

# Stanford Doggo: An Open-Source, Quasi-Direct-Drive Quadruped

Nathan Kau, Aaron Schultz, Natalie Ferrante, Patrick Slade

The authors are with the Department of Mechanical Engineering, Stanford University, Stanford, CA 94305 USA (e-mail: nathankau@stanford.edu).

**Abstract** — This paper presents Stanford Doggo, a quasidirect-drive quadruped capable of dynamic locomotion. This robot matches or exceeds common performance metrics of stateof-the-art legged robots. In terms of vertical jumping agility, a measure of average vertical speed, Stanford Doggo matches the best performing animal and surpasses the previous best robot by 22%. An overall design architecture is presented with focus on our quasi-direct-drive design methodology. The hardware and software to replicate this robot is open-source, requires only hand tools for manufacturing and assembly, and costs less than $3000.

## I. INTRODUCTION

Legged robots provide a highly mobile platform to traverse difficult terrain and are ideal for accomplishing tasks that are repetitive, strenuous, or dangerous. Many state-of-theart legged robots have achieved remarkable feats including high speed locomotion [1], [2], agile maneuvers [3], [4], [5], and traversing difficult terrain [6], [7], [8], [9]. Some designs store energy, such as with a parallel-elastic leg mechanism, to achieve dynamic motion and are not capable of continuously agile motion required to accomplish many tasks [10], [11], [12]. Only robots that carry their power supply and are capable of repeated jumping or locomotion are considered in this work. These platforms take years of development and are frequently expensive, custom designs. 

Many metrics characterize the performance of legged robots: steady velocity during running, jump height, and vertical jumping agility. Vertical jumping agility quantifies how quickly an animal can change its energetic state, approximating the vertical climbing speed through a series of jumps [5], [13]. This metric correlates to locomotion performance as the distance a robot can jump increases its ability to overcome obstacles, improving path-planning capabilities [14]. While current quadruped robots are a popular platform capable of carrying payloads, performing manipulation, and fall recovery, they are unable to match the vertical jumping agility of specialized monopod robots [5], [15] that attempt to emulate the animal with the best jumping agility, the galago (Galago senegalensis) [16]. 

Legged robots require a trade-off between energy efficiency to accomplish their task and sensitivity to safely interact with their environment. Often, robots employ a high reduction gear train to increase the effective torque produced by the motor. This rigid gear train requires compliance to be designed in series with the motor, referred to as a series-elastic actuator [17]. Direct-drive (DD) robots do not employ any speed reduction between the motor and output 
