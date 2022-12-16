// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class DriveTrain {
    SwerveModule[] swerveModules;
    Translation2d[] moduleOffset;
    ShuffleboardTab drivetrainTab;
    double transX;
    double transY;
    double rotation;
    boolean autoDrive;
    public static double[] previousAngle = new double[4];

    // initialize
    // update
    // set
    public DriveTrain init() {
        drivetrainTab = Shuffleboard.getTab("Drivetrain");
        moduleOffset = new Translation2d[] {
                new Translation2d(Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0,
                        Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0),
                new Translation2d(Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0,
                        -Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0),
                new Translation2d(-Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0,
                        -Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0),
                new Translation2d(-Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0,
                        Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0)
        };

        swerveModules = new SwerveModule[] {
                // Mk4i is the module type
                // createFalcon500 means that we have two Falcons on the module
                // the drivetrainTab argument controls "automatically" displaying
                // module info on shuffleboard
                // GearRatio specifies which gearing option is installed - Ask Dom
                // next 3 arguments are Can Bus addresses for the motors and cancoder
                // last arg is the offset of the cancoder angle when the wheel is straight
                // forward
                Mk4iSwerveModuleHelper.createFalcon500(
                        drivetrainTab.getLayout("Front Left Module", BuiltInLayouts.kList)
                                .withSize(2, 2)
                                .withPosition(0, 0),
                        Mk4iSwerveModuleHelper.GearRatio.L2,
                        Constants.FRONT_LEFT_MODULE_DRIVE_MOTOR,
                        Constants.FRONT_LEFT_MODULE_STEER_MOTOR,
                        Constants.FRONT_LEFT_MODULE_STEER_ENCODER,
                        Constants.FRONT_LEFT_MODULE_STEER_OFFSET),

                Mk4iSwerveModuleHelper.createFalcon500(
                        drivetrainTab.getLayout("Front Right Module", BuiltInLayouts.kList)
                                .withSize(2, 2)
                                .withPosition(3, 0),
                        Mk4iSwerveModuleHelper.GearRatio.L2,
                        Constants.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                        Constants.FRONT_RIGHT_MODULE_STEER_MOTOR,
                        Constants.FRONT_RIGHT_MODULE_STEER_ENCODER,
                        Constants.FRONT_RIGHT_MODULE_STEER_OFFSET),

                Mk4iSwerveModuleHelper.createFalcon500(
                        drivetrainTab.getLayout("Back Right Module", BuiltInLayouts.kList)
                                .withSize(2, 2)
                                .withPosition(3, 5),
                        Mk4iSwerveModuleHelper.GearRatio.L2,
                        Constants.BACK_RIGHT_MODULE_DRIVE_MOTOR,
                        Constants.BACK_RIGHT_MODULE_STEER_MOTOR,
                        Constants.BACK_RIGHT_MODULE_STEER_ENCODER,
                        Constants.BACK_RIGHT_MODULE_STEER_OFFSET),

                Mk4iSwerveModuleHelper.createFalcon500(
                        drivetrainTab.getLayout("Back Left Module", BuiltInLayouts.kList)
                                .withSize(2, 2)
                                .withPosition(0, 5),
                        Mk4iSwerveModuleHelper.GearRatio.L2,
                        Constants.BACK_LEFT_MODULE_DRIVE_MOTOR,
                        Constants.BACK_LEFT_MODULE_STEER_MOTOR,
                        Constants.BACK_LEFT_MODULE_STEER_ENCODER,
                        Constants.BACK_LEFT_MODULE_STEER_OFFSET)
        };
        return this;
    }

    public void update() {
        ChassisSpeeds chassisSpeeds = null;
        if (!autoDrive) {
            chassisSpeeds =ChassisSpeeds.fromFieldRelativeSpeeds(transX,
            transY,
            rotation, new Rotation2d(Math.toRadians(-Robot.Navx.getYaw())));
        //     chassisSpeeds = new ChassisSpeeds(transX, transY, rotation);
        } else {
            chassisSpeeds = new ChassisSpeeds(transX,
                    transY, rotation);
        }
        SwerveDriveKinematics swerveDriveKinematics = new SwerveDriveKinematics(moduleOffset);
        SwerveModuleState[] swerveModuleState = swerveDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleState,
                Constants.MAX_VELOCITY_METERS_PER_SECOND);
        for (int i = 0; i < moduleOffset.length; i++) {
            // SwerveModuleState.optimize(swerveModuleState[i],
            // new Rotation2d(swerveModules[i].getSteerAngle()));
            // Comment the following line for calibration...
            if (Math.abs(transX) >= 0.001 || Math.abs(transY) >= 0.001 || Math.abs(rotation) >= 0.001) {

                swerveModules[i].set((swerveModuleState[i].speedMetersPerSecond /
                        Constants.MAX_VELOCITY_METERS_PER_SECOND)
                        * Constants.MAX_VOLTAGE, swerveModuleState[i].angle.getRadians());

                previousAngle[i] = swerveModuleState[i].angle.getRadians();
            } else {
                swerveModules[i].set((swerveModuleState[i].speedMetersPerSecond /
                        Constants.MAX_VELOCITY_METERS_PER_SECOND)
                        * Constants.MAX_VOLTAGE, previousAngle[i]);

            }
        }
        SmartDashboard.getEntry("steerAAngle").setDouble(swerveModuleState[1].angle.getRadians());
    }

    // TODO: transX.getSquared()
    public void set(double transX, double transY, double rotation, boolean autoDrive) {
        this.transX = transX;
        this.transY = transY;
        this.rotation = rotation;
        this.autoDrive = autoDrive;
    }
}
