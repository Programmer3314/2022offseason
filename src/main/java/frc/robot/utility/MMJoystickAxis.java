// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

import edu.wpi.first.wpilibj.Joystick;

/** Add your docs here. */
public class MMJoystickAxis {
    Joystick joystick;
    int axis;
    double deadzone;
    double scale;

    public MMJoystickAxis(int joystickIndex, int axisIndex, double deadzone, double scale) {
        this.joystick = new Joystick(joystickIndex);
        this.axis = axisIndex;
        this.deadzone = deadzone;
        this.scale = scale;
    }

    public double get() {
        double rawAxis = joystick.getRawAxis(axis);

        return getDeadzone(rawAxis)*scale;
    }
    public double getSquared(){
        double rawAxis = joystick.getRawAxis(axis);
        double gotGet = getDeadzone(rawAxis);
        if(gotGet>=0){
            gotGet*=gotGet;
        }
        else{
            gotGet*=-gotGet;
        }
        return gotGet*scale;
    }
    public double getDeadzone(double rawAxis){
        double result;

        if (deadzone <= Math.abs(rawAxis)) {
            result = Math.signum(rawAxis) * (Math.abs(rawAxis) - deadzone) / (1 - deadzone);
        } else {
            result = 0;
        }
        return result;
    }
}
