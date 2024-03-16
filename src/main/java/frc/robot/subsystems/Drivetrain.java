// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Ports.DrivetrainPorts;
import frc.robot.subsystems.modules.MaxSwerveModule;
import frc.robot.utils.SwerveUtils;
import monologue.Logged;
import monologue.Annotations.Log;
import frc.robot.DrivetrainConstants.*;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import java.util.List;

public class Drivetrain extends SubsystemBase implements Logged {
  // Create MaxSwerveModules

  private final MaxSwerveModule frontLeft = new MaxSwerveModule(
      DrivetrainPorts.FRONT_LEFT_DRIVE,
      DrivetrainPorts.FRONT_LEFT_TURNING,
      DriveConstants.FL_CHASSIS_ANGULAR_OFFSET);

  private final MaxSwerveModule frontRight = new MaxSwerveModule(
      DrivetrainPorts.FRONT_RIGHT_DRIVE,
      DrivetrainPorts.FRONT_RIGHT_TURNING,
      DriveConstants.FR_CHASSIS_ANGULAR_OFFSET);

  private final MaxSwerveModule rearLeft = new MaxSwerveModule(
      DrivetrainPorts.REAR_LEFT_DRIVE,
      DrivetrainPorts.REAR_LEFT_TURNING,
      DriveConstants.RL_CHASSIS_ANGULAR_OFFSET);

  private final MaxSwerveModule rearRight = new MaxSwerveModule(
      DrivetrainPorts.REAR_RIGHT_DRIVE,
      DrivetrainPorts.REAR_RIGHT_TURNING,
      DriveConstants.RR_CHASSIS_ANGULAR_OFFSET);

  private final AHRS gyro = new AHRS();

  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(DriveConstants.MODULE_OFFSET);

  // Slew rate filter variables for controlling lateral acceleration
  private double currentRotation = 0.0;
  private double currentTranslationDir = 0.0;
  private double currentTranslationMag = 0.0;

  private SlewRateLimiter magLimiter = new SlewRateLimiter(DriveConstants.MAG_SLEW_RATE);
  private SlewRateLimiter rotLimiter = new SlewRateLimiter(DriveConstants.ROT_SLEW_RATE);

  private double prevTime = WPIUtilJNI.now() * 1e-6;
  private double speedFactor = 1.0; // was 0.3 for school

  // odometry robot pose
  //out of date - swerve drive pose estimator is new
  SwerveDriveOdometry odometry = new SwerveDriveOdometry(
      kinematics,
      Rotation2d.fromDegrees(getAngle()),
      new SwerveModulePosition[] {
          frontLeft.getPosition(),
          frontRight.getPosition(),
          rearLeft.getPosition(),
          rearRight.getPosition()
      });

  private final SwerveDrivePoseEstimator odometryEstimator;
  
  @Log.NT
  private final Field2d field2d = new Field2d();

  private final List<MaxSwerveModule> modules = List.of(frontLeft, frontRight, rearLeft, rearRight);

  /* SYSID INSTANTIATIONS */
  private final SysIdRoutine driveRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(), new SysIdRoutine.Mechanism(
          volts -> modules.forEach(m -> m.setDriveVoltage(volts.in(Units.Volts))),
          null, this));

  private final SysIdRoutine turnRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(), new SysIdRoutine.Mechanism(
          volts -> rearLeft.setTurnVoltage(volts.in(Units.Volts)), null, this));

  private final SysIdRoutine turnAllRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(), new SysIdRoutine.Mechanism(
          volts -> modules.forEach(m -> m.setTurnVoltage(volts.in(Units.Volts))),
          null, this));

  public Drivetrain() {
    zeroHeading();

    odometryEstimator = new SwerveDrivePoseEstimator(
      kinematics,
      gyro.getRotation2d(),
      new SwerveModulePosition[]{
        frontLeft.getPosition(),
        frontRight.getPosition(),
        rearLeft.getPosition(),
        rearRight.getPosition()
      }, 
      new Pose2d(),
      VecBuilder.fill(
        0.1,
        0.1,
        0.1),
      VecBuilder.fill(
        0.9,
        0.9,
        0.9
      ));
  }
  
  /* COMMANDS */
  /**
   * Drives the robot using joystick inputs.
   *
   * @param x A double representing forward speed (y-dir)
   * @param y A double representing sideways speed (x-dir)
   * @param rot A double representing rotation.
   * @param fieldRel A boolean to indicate field relativity.
   * @param rateLim a boolean to indicate rate limit.
   * @param deadband A double representing the joystick deadband
   * 
   * @return The command to drive the robot.
   */
  public Command defaultCmd(double x, double y, double rot, boolean fieldRel, boolean rateLim, double deadband) {
    return this.run(() -> drive( // all joy.get values were prev negative
    MathUtil.applyDeadband(x, deadband),
    MathUtil.applyDeadband(y, deadband),
    MathUtil.applyDeadband(rot, deadband),
    true,
    true));
  }

  /**
   * Resets gyro.
   */
  public Command resetGyroCmd() {
    return this.runOnce(() -> zeroHeading());
  }

  @Override
  public void periodic() {
    // updates periodically
    odometry.update(
        Rotation2d.fromDegrees(getAngle()),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            rearLeft.getPosition(),
            rearRight.getPosition()
        });

    SmartDashboard.putNumber("gyro angle", getAngle());
    // System.out.println("yaw reading" + gyro.getYaw());
    // System.out.println("angle reading " + getAngle());
    // SmartDashboard.putNumber("gyro x", gyroX()); <-
  }

  /** 
   * @return currently-estimated pose of robot
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  // resets the odometry to the specified pose
  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(
        Rotation2d.fromDegrees(gyro.getYaw()),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            rearLeft.getPosition(),
            rearRight.getPosition()}, pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {

    double xSpeedCommanded;
    double ySpeedCommanded;

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral
      // acceleration
      double directionSlewRate;

      if (currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(DriveConstants.DIR_SLEW_RATE / currentTranslationMag);
      }

      else {
        directionSlewRate = 500.0; // some high number that means the slew rate is effectively instantaneous
      }

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, currentTranslationDir);

      if (angleDif < 0.45 * Math.PI) {
        currentTranslationDir = SwerveUtils.StepTowardsCircular(currentTranslationDir, inputTranslationDir,
            directionSlewRate * elapsedTime);
        currentTranslationMag = magLimiter.calculate(inputTranslationMag);
      }

      else if (angleDif > 0.85 * Math.PI) {
        if (currentTranslationMag > 1e-4) { // some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          currentTranslationMag = magLimiter.calculate(0.0);
        }

        else {
          currentTranslationDir = SwerveUtils.WrapAngle(currentTranslationDir + Math.PI);
          currentTranslationMag = magLimiter.calculate(inputTranslationMag);
        }
      }

      else {
        currentTranslationDir = SwerveUtils.StepTowardsCircular(currentTranslationDir, inputTranslationDir,
            directionSlewRate * elapsedTime);
        currentTranslationMag = magLimiter.calculate(0.0);
      }

      prevTime = currentTime;

      xSpeedCommanded = currentTranslationMag * Math.cos(currentTranslationDir);
      ySpeedCommanded = currentTranslationMag * Math.sin(currentTranslationDir);
      currentRotation = rotLimiter.calculate(rot);
    }

    else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      currentRotation = rot;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * DriveConstants.MAX_SPEED * speedFactor;
    double ySpeedDelivered = ySpeedCommanded * DriveConstants.MAX_SPEED * speedFactor;

    // SmartDashboard.putNumber("xspeed drive", xSpeedDelivered);
    // SmartDashboard.putNumber("xspeed drive", xSpeedDelivered);
    // System.out.println("xspeed drive: " + xSpeedDelivered);
    // System.out.println("yspeed drive: " + ySpeedDelivered);

    double rotDelivered = currentRotation * DriveConstants.MAX_ANGULAR_SPEED;
    var swerveModuleStates = kinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                Rotation2d.fromDegrees(getAngle()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.MAX_SPEED);

    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    rearLeft.setDesiredState(swerveModuleStates[2]);
    rearRight.setDesiredState(swerveModuleStates[3]);

  }

  // x formation with wheels -> prevent movement
  public void setX() {
    frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  // sets the swerve ModuleStates.
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.MAX_SPEED);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    rearLeft.setDesiredState(desiredStates[2]);
    rearRight.setDesiredState(desiredStates[3]);
  }

  // resets drive encoders
  public void resetEncoders() {
    frontLeft.resetEncoders();
    rearLeft.resetEncoders();
    frontRight.resetEncoders();
    rearRight.resetEncoders();
    // Arrays.stream(swerveModules).forEach(RevSwerveModule::resetEncoders);
  }

  // zeros heading/resets/calibrates gyro
  public void zeroHeading() {
    gyro.reset();
  }

  // @return the robot's heading in degrees, from -180 to 180
  // returns heading in degrees (-180 to 180)
  public double getHeading() {
    return Rotation2d.fromDegrees(getAngle()).getDegrees();
  }

  // @return The turn rate of the robot (deg/s)
  public double getTurnRate() {
    return gyro.getRate() * (DriveConstants.GYRO_REVERSED ? -1.0 : 1.0);
  }

  public double getYawAxis() {
    return gyro.getYaw();
  }

  public double setYawOffset() {
    gyro.setAngleAdjustment(-90); // need to double check!
    return gyro.getYaw();
  }

  public double getAngle() {
    return -1 * gyro.getAngle();
  }

  public double getPitch() {
    return gyro.getPitch();
  }

  public void slowSpeed() {
    speedFactor = 0.5;
  }

  public void regSpeed() {
    speedFactor = 1;
  }

  /* SPEED FACTOR CMD */
  public Command slowCmd()
  {
    return this.runOnce(() -> slowSpeed());
  }
  public Command regularCmd()
  {
    return this.runOnce(() -> regSpeed());
  }
  
  /* SYSID CMDS */
  public Command driveQuasistatic(SysIdRoutine.Direction direction) {
    return driveRoutine.quasistatic(direction);
  }

  public Command turnQuasistatic(SysIdRoutine.Direction direction) {
    return turnRoutine.quasistatic(direction);
  }

  public Command driveDynamic(SysIdRoutine.Direction direction) {
    return driveRoutine.dynamic(direction);
  }

  public Command turnDynamic(SysIdRoutine.Direction direction) {
    return turnRoutine.dynamic(direction);
  }
}
