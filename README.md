# README #

This README would normally document whatever steps are necessary to get your application up and running.

### What is this repository for? ###

* Quick summary

- This is the control package for trajectory tracking of a Parrot Bebop 2. 

* Version 

- git 1.0

* [Learn Markdown](https://bitbucket.org/tutorials/markdowndemo)

### How do I get set up? ###
## For a complete description of identification, setup and operation, please refer to the pdf manual. ##

* Summary of set up

- Basically, you will need:

1. A Parrot Bebop 2 with model parameters identified and saved on drone_dev. 
2. Configured joystick for takeoff, landing, frame reset and activating controller.
3. For better results, we suggest using cable connection to a repeater. 
4. For accurate indoor results: reflexive markers and a Vicon Motion Tracking System.

* Configuration

- Once the setup is complete, just edit the .yaml file "config/bebopParameters" for assigning a task and tuning controller parameters.

* Dependencies

- bebop_autonomy
- vicon_bridge

* Database configuration
* How to run tests

- Simply edit the .yaml file "config/bebopParameters" for assigning a task and tuning controller parameters.

* Deployment instructions

### Contribution guidelines ###

* Writing tests
* Code review
* Other guidelines

Refer to the pdf manual for a detailed description for operating this package.

### Improvements for next version ###

1. Define default parameters from functions setControlSelect, setSensorSelect and setTrajectory on  initDroneParam();
2. Standardize VERBOSE for all of the controllers;
3. Standardize flags to be bool private variables -->  use get and set to alter;
4. Change IFs to SWITCH-CASE
5. Pass desired linear and angular acceleration on genTrajectory.cpp (this will remove the necessity of knowing wAng
in droneSystem.cpp and also will provide a cleaner solution for the "ident" trajectory)
6. Implement control saturations with a single function on "control()"
7.

### Who do I talk to? ###

* Repo owner or admin
jrsbenevides: jrsbenevides@usp.br

* Other community or team contact
rsinoue: rsinoue@ufscar.br
araujorayza: rayza.araujo@usp.br
