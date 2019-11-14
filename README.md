# RMF Fleet Adapter
This chapter goes into the details of writing a fleet adapter. Recall that we have learnt the following other components:
* Vendor Fleet: An MiR robot that, given a specified `PoseStamped`, is able to autonomously navigate to that location.
* Fleet Manager: Manages the logic that allows for higher level control of vendor fleet(s). In our example, the fleet manager allows the queuing of multiple `PoseStamped`, and handles the sequential movement of the fleet to these poses.
* Traffic Editor: A tool for specifying a universal RMF representation of the operational environment. In particular, the vertices in the map represent 
Waypoints for the vendor fleet to move to.
* Schedule: An RMF model of other agents in the environment. Given this model, we can decide how our agent should best act. For example, by consulting the Schedule, we can decide a feasible sequence of Waypoints, as described in the traffic editor, which will not disturb other fleets.

Now, the final part of the puzzle is the Fleet Adapter. The fleet adapter has the following main functions:
* To implement the logic that combines the knowledge in the Schedule, with the unique `VehicleTraits` of our fleet, in order to generate a sequence of Waypoints to follow through.
* To translate and broadcast the Waypoint sequence, which is an abstract RMF representation, in a representation that our fleet manager is able to process. ( In our case, a series of PoseStamped messages ).
* (There is more stuff, to be added when implemented )

As a result, a fleet adapter is likely to be highly customized to each type of fleet that exists and allows bridging of various systems info a unified world representation. 