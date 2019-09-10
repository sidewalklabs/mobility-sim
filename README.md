# Mobility Simulation

Here lies the code behind our street design simulation [blog post](https://medium.com/sidewalk-talk/https-medium-com-sidewalk-talk-street-sim-33da7e1a8ffb) (Sept 2019).

The XML files in `bau-inputs/` and `proposed-inputs/` are manually configured. They specify the geometry of the nodes, edges, routes (in which we parameterize vehicle types), and street types of our network. Running commands such as the following:

`netconvert -c bau.netccfg`
`netconvert -c proposed.netccfg` 

will generate the "network files" `bau.net.xml` and `proposed.net.xml`. It is not recommended to manually edit these network files; any change to the network should be made through its input files and the commands above should be re-run to produce a new network file. 

Running `sumo-gui -c proposed.sumocfg` or `sumo-gui -c bau.sumocfg` should open up and start the simulation in the SUMO GUI. However, this code base generates trips programmatically via TraCi, so we instead call:

`python3 generateTrips.py proposed` or
`python3 generateTrips.py bau`

to launch the GUI and simulation. See generateTrips.py for more details on how the simulation is run (in particular, how self.spawnrate can be used to effectively increase/decrease trip demand). 

Finally, some metrics (e.g. mean/std for wait times by vehicle type, etc.) can be collected by running 

`python3 stats.py <simtype>`.

## Future Directions

The primary shortcoming of the simulation model as it currently stands is its lack of support for dynamic user assignment -- i.e. the "shortest path" from A to B should not be fixed according to initial costs of each edge, but rather should be dynamically calculated at each step according to live travel times across each edge (which is heavily dependent on congestion). 

Sidewalk's [microsim](https://github.com/sidewalklabs/microsim/blob/master/) repo implements dynamic user reassignment using SUMO's DUAROUTER tool. Notably, in [this file](https://github.com/sidewalklabs/microsim/blob/master/random_generation.py), they generate and populate a random ".trips.xml" file, and use DUAROUTER to dynamically generate route files. It should be straightforward to do implement something similar for this simulation.

For future steps beyond this, refer to Modus's [microsimulation exploration](https://docs.google.com/document/d/1Tgy8h9XqMA7UA5AxusY4WbpgvU0immNiMku3BgsiElA/edit?usp=sharing). 

