# Cooperative Perception: Centralized Approach
# Step A - Data Generation
1. Adjust the `path` value in `initialize_folders.py` at line 31 for the path to generate the data
2. Run `prepare_data/initialize_folders.py` to generate the data

Steps for testing:
1. Run `traCi.py` to generate the data and the scores. 
2. Run `solve_traci.py` to simulate the network optimization process

For Bulk of run: 
1. Run `run_all_simulations.py`
2. Run `solve_all_simulations.py`

Statistics:
1. Run make_plots.py 

Links:
- Vehicle Types:
https://sumo.dlr.de/docs/Tools/Trip.html#automatically_generating_a_vehicle_type

- Random Types:
https://sumo.dlr.de/docs/Simulation/Randomness.html

- Example on random types:
https://sumo-user.narkive.com/qiPPGoIq/using-vtypedistribution
