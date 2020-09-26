# Time-to-Collision(TTC) based Local Navigation for Multiple Agent

## Description

Local collision avoidance plays an important role in multi- agent navigation and planning. This folder conatins implementation of predictive TTC forces approach for local navigation for multi-agent navigation scenerios.

TTC based approach takes forces that depend on relative displacement of the agents at the moment of a collision into consideration. Moreover, the uncertaninty in the sensed velocities of the neighbors using the isotropic formulation has also been incorported in the given implementation.\
More information regarding the "Uncertainity Models for TTC-Based Collision Avoidance" can be found [here](https://www-users.cs.umn.edu/~foro0012/UTTC/uncertainty-models-ttc.pdf) .\
Go in the 8 agents and/or Crowd Crossing folder and type `python simulator.py` to run the simulation.

## Visualization of TTC based approach

1.  [8 Agent Simulation](https://github.com/prateeks97/Motion_Planning/tree/master/Force%20Based%20Local%20Navigation/8%20Agents)
<img  src="https://github.com/prateeks97/Motion_Planning/blob/master/Force%20Based%20Local%20Navigation/8%20Agents/Implementation/8_agents.gif"  width="500"  height="500"/>

2.  [Crowd Crossing Simulation](https://github.com/prateeks97/Motion_Planning/tree/master/Force%20Based%20Local%20Navigation/Crowd%20Crossing)
<img  src="https://github.com/prateeks97/Motion_Planning/blob/master/Force%20Based%20Local%20Navigation/Crowd%20Crossing/Implementation/crowd_crossing.gif"  width="500"  height="500"/>