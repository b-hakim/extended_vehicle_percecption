# Cooperative Perception: Centralized Approach
Generate and load the data:
* Data generation steps in to https://github.com/b-safwat/sumo2vision
* Use timestamp=1

Steps for testing:
* Run `solve_traci.py` to simulate the network optimization process

For Bulk of run: 
* Run `solve_all_simulations.py`

Statistics:
* Run make_plots.py 

Links:
- Vehicle Types:
https://sumo.dlr.de/docs/Tools/Trip.html#automatically_generating_a_vehicle_type

- Random Types:
https://sumo.dlr.de/docs/Simulation/Randomness.html

- Example on random types:
https://sumo-user.narkive.com/qiPPGoIq/using-vtypedistribution

Kindly cite this paper if you are using it:
> @article{hakim2023ccpav,
  title={CCPAV: Centralized cooperative perception for autonomous vehicles using CV2X},
  author={Hakim, Bassel and Sorour, Sameh and Hefeida, Mohamed S and Alasmary, Waleed S and Almotairi, Khaled H},
  journal={Ad Hoc Networks},
  volume={142},
  pages={103101},
  year={2023},
  publisher={Elsevier}
}
