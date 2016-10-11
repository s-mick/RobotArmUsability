# RobotArmUsability #

Software framework for the usability assessment of a robotic arm prosthesis, based on a center-out target-reaching task

This framework was developed during an engineering school six-month internship at [FLOWERS](https://flowers.inria.fr) research team from Inria, in collaboration with [Hybrid](http://www.incia.u-bordeaux1.fr/spip.php?article340) team from INCIA. This work is part of a project addressing the design of robotic arm prostheses and methods to drive these prostheses with physiological signals.

## Development ##

This framework is developed in Python and based on four external libraries:
* [Poppy](https://poppy-project.org) architecture - Robotic device
* [IKPy](https://github.com/Phylliade/ikpy) - Manage the mechanical model and kinematics calculations to drive the robot's endpoint
* [PsychoPy](http://www.psychopy.org/) - Screen display, event handling and timing
* [PyDAQmx](https://pythonhosted.org/PyDAQmx/) - Connection and data retrieving from National Instruments acquisition board

Software was developed and tested on Python 2.7 for Windows. Psychopy requires a 32-bit Python version, and is not compatible with 3.X. PyDAQmx is only available for Windows.

## Experiment ##

The experiment takes place as a way to assess the usability of various control modes that can be used to drive the robot. Several measurements, in the form of quantitative and qualitative data, provide results to compare these different modes on aspects such as movement speed, accuracy, user satisfaction, comfort…

The experiment is built over a center-out target-reaching task: the robot endpoint starts at a default position, and the subject has to make it move so it reaches a target and dwells on it for a short time. Measurements of movement time and accuracy are performed during this task and provide an objective evaluation of the performance that can be achieved with each of the available modes.

## Conduct the experiment ##

* `gripperarm.py` is a small module interfacing the physical Poppy robot with IKPy features. It is imported by every script using the physical robot.
* Run `ExpTargetOrder.py` to generate a target order based on a set of targets. Target positions and order will be saved as a `.npz` archive file.
* Run `demoTrial.py` to illustrate the task to a subject
* Run `GoalDriving.py` to conduct one experimental sequence. Raw force data, trajectories and metrics values will be saved as `.npz` archive files. Only this script requires PsychoPy.
* Run `statExploit.py` to perform statistical analysis on the metrics values generated during the experimentals sequences
* Run `trajPlot` to display cross sections of the endpoint trajectories performed during an experiment by one subject

## Documentation ##

The source files contain a basic inline documentation in English. The documents describing the project, experiment and protocol are available only in French. A detailed forum post on these works is available [here](https://forum.poppy-project.org/t/real-time-control-of-a-poppy-right-arm-with-a-force-sensor-and-usability-assessment-of-several-control-modes/2633)
