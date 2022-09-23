// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

import frc.robot.Robot;

public class MMStateMachine {
    double stateStartSeconds;
    double secondsInState;
    long stateStartCycles;
    long cyclesInState;

    MMStateMachineState currentState;
    boolean firstUpdate;

    public MMStateMachine setInitial(MMStateMachineState initialState) {
        currentState = initialState;
        firstUpdate = true;
        return this;
    }

    public MMStateMachine update() {
        secondsInState = Robot.now - stateStartSeconds;
        cyclesInState = Robot.cycle - stateStartCycles;
        if (!firstUpdate) {
            MMStateMachineState nextState = currentState.calcNextState();
            if (currentState != nextState) {
                currentState.transistionFrom(nextState);
                stateStartCycles = Robot.cycle;
                stateStartSeconds = Robot.now;
                secondsInState = 0;
                cyclesInState = 0;
                nextState.transitionTo(currentState);
                currentState = nextState;
            }
        } else {
            firstUpdate = false;
        }
        currentState.doState();
        return this;
    }
}