// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

public abstract class MMStateMachineState {
    String name;

    public MMStateMachineState(String name) {
        this.name = name;
    }
    public abstract MMStateMachineState calcNextState();
    public abstract void transitionTo(MMStateMachineState previousState);
    public abstract void transistionFrom(MMStateMachineState nextState);
    public abstract void doState();
}

