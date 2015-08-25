# Q-Learning-Simulation

Author: Usama Yaseen

g++ *.cpp -o main

I worked on this project as a part of my inter-disciplinary project at Technical University of Munich. Due to permission issue I cannot share the portion of code implementing Central Pattern Generator (CPG), therefore that portion is being cover-up by simulating dummy motion patterns from dummy sensor values which are then passed to the Q-learning code, which btw doesn't distinguish between dummy motion patterns or the real motion patterns. Also, the actual simulation was performed in webots, however this dummy (only CPG & sensor values part is dummy :-) ) implementation does not have any dependecy on webots and require only g++ compiler.

Abstract:
Locomotion is a central task for a humanoid robot to interact in a human environment. Different models for robot locomotion have been proposed in order to improve the robustness for environmental changes. Existing neurobiological studies have revealed that rhythmic motor patterns are controlled by neural oscillators referred as central pattern generators (CPGs). Dr. John Nassour and others proposed a multi-layer multi-pattern central pattern generator[1] in order to generate different motion patterns: periodic and aperiodic ones. This project involves utilizing the CPG developed earlier to deal with the problem of external perturbation. Whenever humans or animals are hit by an external object, their body generates a response action which prevents them from falling or at-least this response action tries to minimize the effect of the perturbation, this response action is spontaneous and often un-noticeable by us. This response action is generally a specific movement, which in-fact is generation of some pattern, and in mammals usually this is done by CPG. The idea is to apply reinforcement learning techniques to find out those 'speical actions' which will prevent the robot from falling when encountered with foreign disturbance, so basically it's learning over action space with Q-learning.

[1] Nassour J. and , Hénaff P. and Benouezdou F. and Cheng G. , "Multi-layered multi-pattern CPG for adaptive locomotion of humanoid robots", Biological Cybernetics, June 2014, Volume 108, Issue 3, pp 291-303
