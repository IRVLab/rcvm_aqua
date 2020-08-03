# rcvm_aqua

rcvm_aqua provides the Aqua specific implementation of RCVM. You need rcvm_core to use this package, as rcvm_core provides the service definitions for the services implemented in this package.

To use, you can either call the launch file for a specific kineme anytime you want it, or you can start the server and call a service.

To use a launch file:

    roslaunch rcvm_aqua [kineme_name.launch]

To use the server manually:

    rosrun rcvm_aqua server.py
    rosservice call /rcvm/[kineme_name] [service-args]


The rcvm_ac.py is heavily based on the autopilot client from the aquaautopilot package, by David Meger.

### References
For more general information on RCVM: [IRV Lab Project Page](https://irvlab.dl.umn.edu/projects/robot-communication-motion)
For the ICRA 19 paper (Unreal Engine Simulation of Aqua RCVM): [ICRA19 Paper](https://ieeexplore.ieee.org/abstract/document/8793491)
For an arXiv paper describing the first version of Aqua, Matrice, and Turtlebot RCVM: [arXiv Paper](https://arxiv.org/abs/1903.03134)

If citing something related to RCVM, please use the following reference [(.bib file)](https://raw.githubusercontent.com/fultonms/publications/master/rcvm_icra18.bib):


    Fulton, Michael, Chelsey Edge, and Junaed Sattar. "Robot Communication Via Motion: Closing the Underwater Human-Robot Interaction Loop." 
    2019 International Conference on Robotics and Automation (ICRA). IEEE, 2019.