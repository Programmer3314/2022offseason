package frc.robot.utility;
// Copyright (c) FIRST and other WPILib contributors.

import com.ctre.phoenix.motorcontrol.NeutralMode;

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;




/** Add your docs here. */
public class MMSwerveModule {
    private final WPI_TalonFX driveMotorController;
    private final WPI_TalonFX turnMotorController;
    private final WPI_CANCoder magneticCanCoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffset;
    private final PIDController turnPidController;

    public MMSwerveModule(int driveMotorCanId, int turnMotorCanId, int magneticCanCoderId, boolean driveMotorReversed,
            boolean turnMotorReversed, int absoluteEncoderId, double absoluteEncoderOffset,
            boolean absoluteEncoderReversed) {
        this.absoluteEncoderOffset = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        magneticCanCoder = new WPI_CANCoder(magneticCanCoderId);

        driveMotorController = new WPI_TalonFX(driveMotorCanId);
        driveMotorController.configFactoryDefault();
        driveMotorController.setNeutralMode(NeutralMode.Brake);
        driveMotorController.setInverted(driveMotorReversed);

        turnMotorController = new WPI_TalonFX(turnMotorCanId);
        turnMotorController.configFactoryDefault();
        turnMotorController.setNeutralMode(NeutralMode.Brake);
        turnMotorController.setInverted(turnMotorReversed);
        turnPidController = new PIDController(.65, 0, 0);
        turnPidController.enableContinuousInput(-Math.PI, Math.PI);
    
    }

}

public double getDrivePosition(){
    return driveMotorController.getSelectedSensorPosition()*Constants.driveTicksToMeters;
}
public double getTurningPosition(){
    //TODO simplify angle, with minimalangle or wpiLib version
    return turnMotorController.getSelectedSensorPosition()*Constants.turnTicksToRadians;
}
public double getDriveVelocity(){
    return 0;
}
public double getTurningVelocity(){
    return 0;
}
public double getAbsoluteEncoderRad(){
    return 0;
}