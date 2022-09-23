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
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.utility.MMJoystickAxis;

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

  public static ShuffleboardTab drivetrainTab;

  SwerveModule[] swerveModules;
  Translation2d[] moduleOffset;

  // TODO: update these values
  public static long cycle = 0;
  public static double now = 0;

  // TODO: Add Navx (for field oriented driving)

  @Override
  public void robotInit() {
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

    // TODO: !!!!!! Fix these initializations. Why do they show as not used???
    MMJoystickAxis chassisX = new MMJoystickAxis(Constants.DriverController, Constants.ChassisXAxis, .1,
        -Constants.MAX_VELOCITY_METERS_PER_SECOND);
    MMJoystickAxis chassisY = new MMJoystickAxis(Constants.DriverController, Constants.ChassisYAxis, .1,
        -Constants.MAX_VELOCITY_METERS_PER_SECOND);
    MMJoystickAxis chassisR = new MMJoystickAxis(Constants.DriverController, Constants.ChassisXAxis, .1,
        -Constants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND);

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

  @Override
  public void teleopPeriodic() {

    // TODO: Implement control of chassis
    // doneish-get 3-axis data, using the MMJoystickAxis class
    //           from live code to allow scaling and deadzone
    // done-construct a SwerveDriveKinematics object
    // done-construct a ChassisSpeeds object
    // done-get SwerveDriveStatus objects
    // done-set SwerveModule values
    // done-add Optimization (so wheels don't turn more than needed)
    // add desaturation (so that no wheel exceeds full throttle)
    // SwerveDriveKinematics.normalizeWheelSpeeds(states,
    // MAX_VELOCITY_METERS_PER_SECOND);

    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(chassisX.get(), chassisY.get(), chassisR.get());
    SwerveDriveKinematics swerveDriveKinematics = new SwerveDriveKinematics(moduleOffset);
    SwerveModuleState[] swerveModuleState = swerveDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    for (int i = 0; i < 4; i++) {
      SwerveModuleState.optimize(swerveModuleState[i], new Rotation2d(swerveModules[i].getSteerAngle()));
      swerveModules[i].set((swerveModuleState[i].speedMetersPerSecond / Constants.MAX_VELOCITY_METERS_PER_SECOND)
          * Constants.MAX_VOLTAGE, swerveModuleState[i].angle.getRadians());
    }
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
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
