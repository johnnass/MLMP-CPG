# MLMP-CPG

Multi-layered multi-pattern CPG for adaptive locomotion of humanoid robots

#Abstract

In this paper, we present an extended mathematical model of the central pattern generator (CPG) in the spinal cord. The proposed CPG model is used as the underlying low-level controller of a humanoid robot to generate various walking patterns. Such biological mechanisms have been demonstrated to be robust in locomotion of animal. Our model is supported by two neurophysiological studies. The first study identified a neural circuitry consisting of a two-layered CPG, in which pattern formation and rhythm generation are produced at different levels. The second study focused on a specific neural model that can generate different patterns, including oscillation. This neural model was employed in the pattern generation layer of our CPG, which enables it to produce different motion patterns-rhythmic as well as non-rhythmic motions. Due to the pattern-formation layer, the CPG is able to produce behaviors related to the dominating rhythm (extension/flexion) and rhythm deletion without rhythm resetting. The proposed multi-layered multi-pattern CPG model (MLMP-CPG) has been deployed in a 3D humanoid robot (NAO) while it performs locomotion tasks. The effectiveness of our model is demonstrated in simulations and through experimental results.


#Link to the paper: 

https://link.springer.com/article/10.1007%2Fs00422-014-0592-8

#Linke to the video: 

https://www.youtube.com/watch?v=RJ8hlgq0vos



# The sorce code provided here generates rhythmic walking patterns on Nao humanoid robot.

#Files: 

"MLMPCPG.py": all CPG classes and functions.

"mainNao.py": main control loop to control nao humanoid robot.

"MyNao.txt" the joints' name and the mechanical limits. 

"MyNaoPsitiveAngle_E_or_F.txt": the positive rotation either extension "E" or flexion "F".

"SetTiming.py": the time stamps for injected current puls. 

"NAOMotor.py": robots joints labels. 

