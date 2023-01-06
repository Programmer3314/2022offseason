// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
//FOR OFFSETS, looking at the bottom of the robot, in order to rotate the wheels clockwise --> positive

public class Constants {
    public static final double CHASSIS_LENGTH_IN_INCHES = 24.5;
    public static final double CHASSIS_WIDTH_IN_INCHES =24.5;

    public static final double WHEEL_OFFSET_IN_INCHES = 2.625;

    public static final double DRIVETRAIN_TRACKWIDTH_METERS = .0254*(CHASSIS_WIDTH_IN_INCHES-(WHEEL_OFFSET_IN_INCHES*2));
    public static final double DRIVETRAIN_WHEELBASE_METERS = .0254*(CHASSIS_LENGTH_IN_INCHES-(WHEEL_OFFSET_IN_INCHES*2));


    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 2;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 1;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 1;
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(173+180+37);

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 4;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 3;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 2;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(264.45+9);

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 8;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 7;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 4;
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(238-180);

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 6;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 5;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 3;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(228.5-180);

    public static final double MAX_VOLTAGE = 12.0;
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 4.14528; // With the L2... 4.96824
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
        Math.hypot(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0);
  
    public static final int ChassisXAxis = 1;
    public static final int ChassisYAxis = 0;
    public static final int ChassisRAxis =4;
    public static final int DriverController=4;

    // public static final ModuleConfiguration MK4I_L2 = new ModuleConfiguration(
//             0.10033, -wheel diameter(meters)
//             (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0), -drive reduction
//             true,-driveIsInverted
//             (14.0 / 50.0) * (10.0 / 60.0),-steer Reduction
//             false-steerIsInverted
//     );

    public static final int falconTicksPerRev=2048;
    public static final double wheelDiameter=.10033;
    public static final double driveReduction=(14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);
    public static final boolean driveIsInverted=true;
    public static final double steerReduction=(14.0 / 50.0) * (10.0 / 60.0);
    public static final boolean steerIsInverted=false;
    public static final double wheelCircumference=(wheelDiameter/2)*Math.PI;
    public static final double driveTicksToMeters=(1.0/falconTicksPerRev)*driveReduction*wheelCircumference;
    public static final double turnTicksToRadians=(1.0/falconTicksPerRev)*steerReduction*2*Math.PI;
}
