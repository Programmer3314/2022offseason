// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utility.MMJoystickAxis;
import frc.robot.utility.MMSwerveDriveKinematics;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
        public static MMJoystickAxis chassisX;
        public static MMJoystickAxis chassisY;
        public static MMJoystickAxis chassisR;

        public static double target;
        public static double absoluteNavX;

        public static double incrementPower;

        public static AHRS Navx;

        public static Joystick driverJoystick;

        public static ShuffleboardTab drivetrainTab;

        public static SwerveModule[] swerveModules;
        Translation2d[] moduleOffset;

        // TODO: (later) update these values
        public static long cycle = 0;
        public static double now = 0;

        @Override
        public void robotInit() {
                Navx = new AHRS(Port.kMXP);

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
                                                                .withSize(2, 4)
                                                                .withPosition(0, 0),
                                                Mk4iSwerveModuleHelper.GearRatio.L2,
                                                Constants.FRONT_LEFT_MODULE_DRIVE_MOTOR,
                                                Constants.FRONT_LEFT_MODULE_STEER_MOTOR,
                                                Constants.FRONT_LEFT_MODULE_STEER_ENCODER,
                                                Constants.FRONT_LEFT_MODULE_STEER_OFFSET),

                                Mk4iSwerveModuleHelper.createFalcon500(
                                                drivetrainTab.getLayout("Front Right Module", BuiltInLayouts.kList)
                                                                .withSize(2, 4)
                                                                .withPosition(3, 0),
                                                Mk4iSwerveModuleHelper.GearRatio.L2,
                                                Constants.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                                                Constants.FRONT_RIGHT_MODULE_STEER_MOTOR,
                                                Constants.FRONT_RIGHT_MODULE_STEER_ENCODER,
                                                Constants.FRONT_RIGHT_MODULE_STEER_OFFSET),

                                Mk4iSwerveModuleHelper.createFalcon500(
                                                drivetrainTab.getLayout("Back Right Module", BuiltInLayouts.kList)
                                                                .withSize(2, 4)
                                                                .withPosition(3, 5),
                                                Mk4iSwerveModuleHelper.GearRatio.L2,
                                                Constants.BACK_RIGHT_MODULE_DRIVE_MOTOR,
                                                Constants.BACK_RIGHT_MODULE_STEER_MOTOR,
                                                Constants.BACK_RIGHT_MODULE_STEER_ENCODER,
                                                Constants.BACK_RIGHT_MODULE_STEER_OFFSET),

                                Mk4iSwerveModuleHelper.createFalcon500(
                                                drivetrainTab.getLayout("Back Left Module", BuiltInLayouts.kList)
                                                                .withSize(2, 4)
                                                                .withPosition(0, 5),
                                                Mk4iSwerveModuleHelper.GearRatio.L2,
                                                Constants.BACK_LEFT_MODULE_DRIVE_MOTOR,
                                                Constants.BACK_LEFT_MODULE_STEER_MOTOR,
                                                Constants.BACK_LEFT_MODULE_STEER_ENCODER,
                                                Constants.BACK_LEFT_MODULE_STEER_OFFSET)
                };

                chassisX = new MMJoystickAxis(Constants.DriverController, Constants.ChassisXAxis, .05,
                                -Constants.MAX_VELOCITY_METERS_PER_SECOND / 2);
                chassisY = new MMJoystickAxis(Constants.DriverController, Constants.ChassisYAxis, .05,
                                -Constants.MAX_VELOCITY_METERS_PER_SECOND / 2);
                chassisR = new MMJoystickAxis(Constants.DriverController, Constants.ChassisRAxis, .05,
                                -Constants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND / 2);
                driverJoystick = new Joystick(Constants.DriverController);
        }

        @Override
        public void robotPeriodic() {
        }

        @Override
        public void autonomousInit() {
        }

        @Override
        public void autonomousPeriodic() {
        }

        @Override
        public void teleopInit() {
        }

        public double minimalAngle(double angle) {
                return (((((angle + 180) % 360) + 360) % 360) - 180);
        }

        @Override
        public void teleopPeriodic() {
                // TODO: try this with the minus inside the ()
                // conceptually it's the NavX that is backwards
                // not the result.
                absoluteNavX = -minimalAngle(Navx.getAngle());
                if (driverJoystick.getRawButton(1)) {
                        Navx.reset();
                }

                // TODO: (3) Display the ChassisX, Y, and R values on shuffle board.
                SmartDashboard.getEntry("NavX Angle").setDouble(absoluteNavX);
                double rotation;
                rotation = chassisR.getSquared();
                double error;
                error = minimalAngle(absoluteNavX - target);

                if (driverJoystick.getRawButton(3)) {
                        rotation = 0;
                        double Margin = 1.25;
                        if (error > Margin) {
                                rotation = error / -60;
                                if (rotation > -1) {
                                        rotation = -1;

                                }
                        }
                        if (error < -Margin) {
                                rotation = error / -60;
                                if (rotation < 1) {
                                        rotation = 1;
                                }
                        }
                }
                if (driverJoystick.getRawButton(2)) {
                        target += .4;
                }
                // TODO: make method to adjust any angle from 360 to 180 based.

                SmartDashboard.getEntry("Rotation").setDouble(rotation);
                SmartDashboard.getEntry("Target").setDouble(target);

                

                ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(chassisX.getSquared(),
                                chassisY.getSquared(),
                                rotation, new Rotation2d(Math.toRadians(-Navx.getYaw())));
                // ChassisSpeeds chassisSpeeds = new ChassisSpeeds(chassisX.get(),
                // chassisY.get(), chassisR.get());
                MMSwerveDriveKinematics swerveDriveKinematics = new MMSwerveDriveKinematics(moduleOffset);
                SwerveModuleState[] swerveModuleState = swerveDriveKinematics.toSwerveModuleStates(chassisSpeeds);
                SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleState,
                                Constants.MAX_VELOCITY_METERS_PER_SECOND);
                for (int i = 0; i < moduleOffset.length; i++) {
                        SwerveModuleState.optimize(swerveModuleState[i],
                                        new Rotation2d(swerveModules[i].getSteerAngle()));
                        // Comment the following line for calibration...
                        swerveModules[i].set((swerveModuleState[i].speedMetersPerSecond /
                                        Constants.MAX_VELOCITY_METERS_PER_SECOND)
                                        * Constants.MAX_VOLTAGE, swerveModuleState[i].angle.getRadians());
                }
                SmartDashboard.getEntry("steerAAngle").setDouble(swerveModuleState[1].angle.getRadians());
        }

        @Override
        public void disabledInit() {
        }

        @Override
        public void disabledPeriodic() {
        }

        @Override
        public void testInit() {
        }

        @Override
        public void testPeriodic() {//MINIMUM DRIVE=.51-MINIMUM TURN=
                SmartDashboard.getEntry("Voltage").setDouble(incrementPower);

                if (driverJoystick.getRawButtonReleased(1)) {
                        incrementPower += .1;
                }
                if (driverJoystick.getRawButtonReleased(2)) {
                        incrementPower -= .1;
                }
                if (driverJoystick.getRawButton(4)) {
                        Navx.reset();
                }

                // TODO: (1) Temporarily switch to non-field centric for initial testing
                ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(chassisX.getSquared(),
                                chassisY.getSquared(),
                                0, new Rotation2d(Math.toRadians(-Navx.getYaw())));
                // ChassisSpeeds chassisSpeeds = new ChassisSpeeds(chassisX.get(),
                // chassisY.get(), chassisR.get());
                MMSwerveDriveKinematics swerveDriveKinematics = new MMSwerveDriveKinematics(moduleOffset);
                SwerveModuleState[] swerveModuleState = swerveDriveKinematics.toSwerveModuleStates(chassisSpeeds);
                SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleState,
                                Constants.MAX_VELOCITY_METERS_PER_SECOND);
                for (int i = 0; i < moduleOffset.length; i++) {
                        SwerveModuleState.optimize(swerveModuleState[i],
                                        new Rotation2d(swerveModules[i].getSteerAngle()));
                        // TODO: (2) Please recalibrate and check offsets
                        // Comment the following line for calibration...
                        swerveModules[i].set(0.51, incrementPower);
                }
        }

        @Override
        public void simulationInit() {
        }

        @Override
        public void simulationPeriodic() {
        }
}
