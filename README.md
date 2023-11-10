Cyclistsocialforce: Modified Social Forces for Cyclists with Realistic Dynamics in SUMO
==============================

This is a working repostory for a package that implements a modified social force model for cyclists. Instead of accelerations, our social forces represent the preferred velocities of a cyclist to their destination and around obstacles. This allows to introduce a 2D kinematic bicycle model and two controllers for acceleration and steer. The original social force model for pedestrians was introduced by Helbing and Molnár (1995). Our model uses the separation into tactical and operational behavior introduced by Twaddle (2017).

The model supports co-simulation with [Eclipse SUMO ](https://eclipse.dev/sumo/) via sumolib and the TraCI interface. 

The model is developed for our contribution to the [2023 Bicycle and Motorcycle Dynamics Conference, 18-20 October 2023, Delft, Netherlands](https://dapp.orvium.io/deposits/649d4037c2c818c6824899bd/view), in the context of my PhD project at TU Delft. Refer to our extended abstract for more explanation. If you use this model in your research, please cite it as indicted below. 

### Disclaimer

The package is under development. It may contain bugs and sections of unused or insensible code. Major changes to this package are planned for the time to come. A proper API documentation is still missing. Refer to the demos for examples how to use this model.

## Installation

1. Install Eclipse SUMO ([instructions](https://sumo.dlr.de/docs/Installing/index.html)). Make sure that the SUMO_HOME path variable is set correctly. 

2. Clone this repository. 
   
   ```
   git clone  https://github.com/chrismo-schmidt/cyclistsocialforce.git
   ```

3. Install the package and it's dependencies. Refer to `pyproject.toml` for an overview of the dependencies. 
   
   ```
   cd ./cyclistsocialforce
   pip install . 
   ```

## Demos

The package comes with two demos. The first demo shows a simple interaction between two cyclists. The script is pre-coded with an encroachment confclict. Running the script shows a produces an animation of the interaction and a plot of the force fields and potentials of the two cyclists.  The second demo shows co-simulation of an intersection with SUMO. It launches the SUMO GUI and creates a small scenario of a four-legged intersection with random bicycle demand. On the road segments, cyclists are controlled by SUMO. As soon as cyclists enter the intersection area, the social force model takes over control.  Movements are synchronized between SUMO and the social force model by using the TraCI interface. 

**Running the demos:**

Switch to the demo directroy.

```
cd ./demo
```

Run the standalone demo. Optionally set the `--save` flag to save a pdf of the potential and force plots to the `./demo/output/` folder.

```
python demoCSFstandalone.py
```

Run the SUMO demo. After executing the line below, the SUMO GUI and matplotllib figure opens. To start the simulation, press the 'play' button in the SUMO GUI. To end it, press 'stop'.

```
python demoCSFxSUMO.py
```

## Authors

- Christoph M. Schmidt, c.m.schmidt@tudelft.nl

License
--------------------

This package is licensed under the terms of the [MIT license](https://github.com/chrismo-schmidt/cyclistsocialforce/blob/main/LICENSE).

## Citation

If you use this model in your research, please cite it in your publications as:

Schmidt, C., Dabiri, A., Schulte, F., Happee, R. & Moore, J. (2023). Essential Bicycle Dynamics for Microscopic Traffic Simulation: An Example Using the
Social Force Model. The Evolving Scholar - BMD 2023, 5th Edition. [https://doi.org/10.59490/649d4037c2c818c6824899bd](https://doi.org/10.59490/649d4037c2c818c6824899bd) 

## Bibliography

Helbing, D., & Molnár, P. (1995). Social force model for pedestrian dynamics. *Physical Review E*, *51*(5), 4282–4286. https://doi.org/10.1103/PhysRevE.51.4282

Twaddle, H. (2017). *Development of tactical and operational behaviour models for bicyclists based on automated video data analysis* [PhD Thesis]. Technische Universität München. https://mediatum.ub.tum.de/?id=1366878

## Project Organization

```
.
├── pyproject.toml
├── LICENSE
├── README.md
├── docs
│   └── figures
├── demo
│   ├── config
│   └── output
└── src
    └── cyclistsocialforce
```
