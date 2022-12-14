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
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
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

        public static DriveTrain driveTrain;

        public static NetworkTableInstance nt;
        public static NetworkTable photonVision;
        public static NetworkTableEntry cameraYaw;
        public static NetworkTableEntry cameraHasTarget;

        public static NetworkTableEntry cameraPitch;

        public static double target;
        public static double absoluteNavX;
        public static boolean hasTarget;
        public static boolean autoDrive;

        public static double incrementPower;

        public static AHRS Navx;

        public static Joystick driverJoystick;

        // TODO: (later) update these values
        public static long cycle = 0;
        public static double now = 0;

        @Override
        public void robotInit() {
                Navx = new AHRS(Port.kMXP);

                nt = NetworkTableInstance.getDefault();
                photonVision = nt.getTable("photonvision/mmal_service_16.1");
                cameraYaw = photonVision.getEntry("targetYaw");
                cameraPitch = photonVision.getEntry("targetPitch");
                cameraHasTarget = photonVision.getEntry("hasTarget");

                chassisX = new MMJoystickAxis(Constants.DriverController, Constants.ChassisXAxis, .05,
                                -Constants.MAX_VELOCITY_METERS_PER_SECOND / 2);
                chassisY = new MMJoystickAxis(Constants.DriverController, Constants.ChassisYAxis, .05,
                                -Constants.MAX_VELOCITY_METERS_PER_SECOND / 2);
                chassisR = new MMJoystickAxis(Constants.DriverController, Constants.ChassisRAxis, .05,
                                -Constants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND / 2);
                driverJoystick = new Joystick(Constants.DriverController);

                driveTrain = new DriveTrain().init();
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
                Navx.reset();
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
                hasTarget = cameraHasTarget.getBoolean(false);
                autoDrive = hasTarget && driverJoystick.getRawButton(3);
                double camY = cameraYaw.getDouble(0);
                double camPitch = cameraPitch.getDouble(0);
                double rotation;
                double robotY;
                robotY = chassisY.getSquared();
                double robotX;
                rotation = chassisR.getSquared();
                robotX = chassisX.getSquared();
                double yawError;
                // error = minimalAngle(absoluteNavX - target);
                yawError = camY;
                double yawKP = -.05;// .0325
                double pitchKP = -.0625;
                double desiredPitch = -9;
                double pitchError = camPitch - desiredPitch;
                SmartDashboard.getEntry("Error").setDouble(yawError);
                if (autoDrive) {
                        rotation = 0;
                        double Margin = 1.25;
                        // if (error > Margin) {
                        // rotation = error * kp;
                        // if (rotation > -1) {
                        // rotation = -1;

                        // }
                        // }
                        // if (error < -Margin) {
                        // rotation = error * kp;
                        // if (rotation < 1) {
                        // rotation = 1;
                        // }
                        // }
                        rotation = yawError * yawKP;
                        robotX = pitchError * pitchKP;
                        robotY = 0;
                }
                // if (driverJoystick.getRawButton(2)) {
                // target += .4;
                // }
                // TODO: make method to adjust any angle from 360 to 180 based.

                SmartDashboard.getEntry("Rotation").setDouble(rotation);
                SmartDashboard.getEntry("Target").setDouble(target);
                driveTrain.set(robotX, robotY, rotation, autoDrive);
                driveTrain.update();
                // MMSwerveDriveKinematics swerveDriveKinematics = new
                // MMSwerveDriveKinematics(moduleOffset);

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
        public void testPeriodic() {// MINIMUM DRIVE=.51-MINIMUM TURN=
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
                // MMSwerveDriveKinematics swerveDriveKinematics = new
                // MMSwerveDriveKinematics(moduleOffset);
                // SwerveDriveKinematics swerveDriveKinematics = new
                // SwerveDriveKinematics(moduleOffset);
                // SwerveModuleState[] swerveModuleState =
                // swerveDriveKinematics.toSwerveModuleStates(chassisSpeeds);
                // SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleState,
                // Constants.MAX_VELOCITY_METERS_PER_SECOND);
                // for (int i = 0; i < moduleOffset.length; i++) {
                // // SwerveModuleState.optimize(swerveModuleState[i],
                // // new Rotation2d(swerveModules[i].getSteerAngle()));
                // // TODO: (2) Please recalibrate and check offsets
                // // Comment the following line for calibration...
                // swerveModules[i].set(0.51, incrementPower);
                // }
        }

        @Override
        public void simulationInit() {
        }

        @Override
        public void simulationPeriodic() {
        }
}
