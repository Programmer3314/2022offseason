# 2022offseason:
The initial plan is to produce a small swerve drive practice bot. 
## Primary Goals:
- Orient new programmers.
- Implement basic swerve drive control.
- Evaluate edge case scenerios such as wheel snapping with zero inputs and autonomous steer only operation and replace WPILIB classes if necessary. 
- Add a vision system of some sort to start practicing with the new vision approach using AprilTags. This may not resemble the actual system used in season. 
- Migrate basic code to robust code structures using SubAssemblies to organize code following the form of the robot.


## Additional Goals:
- Try new approach to coding state machines using StateMachine and StateMachineState classes. This will move code from the prior state machine class into descrete StateMachineState class objects. 
This has the advantage of grouping all code related to the state in one place. There may also be improved efficiency. Code size may grow, but it may be simpler. 
This is an experiment to determine which coding style feels better. 


