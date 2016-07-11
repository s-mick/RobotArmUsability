# RobotArmUsability #

Software framework for the usability assessment of a robotic arm prosthesis, based on a center-out target-reaching task

This framework was developed during an engineering school six-month internship at [FLOWERS](https://flowers.inria.fr) research team from Inria, in collaboration with [Hybrid](https://u-bordeaux.fr) team from INCIA. This work is part of a project addressing the design of robotic arm prostheses and methods to drive these prostheses with physiological signals.

## Development ##

This framework is developed in Python and based on four external libraries:
* [Poppy](https://poppy-project.org) architecture - Robotic device
* [IKPy](https://github.com/Phylliade/ikpy) - Manage the mechanical model and kinematics calculations to drive the robot's endpoint
* [PsychoPy](https://www.psychopy.org/) - Screen display, event handling and timing
* [PyDAQmx](https://pythonhosted.org/PyDAQmx/) - Connection and data retrieving from National Instruments acquisition board

## Experiment ##

The experiment takes place as a way to assess the usability of various control modes that can be used to drive the robot. Several measurements, in the form of quantitative and qualitative data, provide results to compare these different modes on aspects such as movement speed, accuracy, user satisfaction, comfort…

The experiment is built over a center-out target-reaching task: the robot endpoint starts at a default position, and the subject has to make it move so it reaches a target and dwells on it for a short time.

## Documentation ##

The source files contain a basic inline documentation in English. The documents describing the project, experiment and protocol are available only in French. A detailed forum post on these works is available [here]()
