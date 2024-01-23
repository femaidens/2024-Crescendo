// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.*;
import frc.robot.Ports.DrivetrainPorts;
import frc.robot.utils.SwerveUtils;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RevisedDrivetrain extends SubsystemBase {
  // Create MAXSwerveModules

  private final MaxSwerveModule[] modules = new MaxSwerveModule[] {
    new MaxSwerveModule(
      DrivetrainPorts.FRONT_LEFT_DRIVE,
      DrivetrainPorts.FRONT_LEFT_TURNING,
      DrivetrainConstants.DriveConstants.FL_CHASSIS_ANGULAR_OFFSET),
    new MaxSwerveModule(
      DrivetrainPorts.FRONT_RIGHT_DRIVE,
      DrivetrainPorts.FRONT_RIGHT_TURNING,
      DrivetrainConstants.DriveConstants.FL_CHASSIS_ANGULAR_OFFSET),
    new MaxSwerveModule(
      DrivetrainPorts.REAR_LEFT_DRIVE,
      DrivetrainPorts.REAR_LEFT_TURNING,
      DrivetrainConstants.DriveConstants.FL_CHASSIS_ANGULAR_OFFSET),
    new MaxSwerveModule(
      DrivetrainPorts.REAR_RIGHT_DRIVE,
      DrivetrainPorts.REAR_RIGHT_TURNING,
      DrivetrainConstants.DriveConstants.FL_CHASSIS_ANGULAR_OFFSET)
  };
/*
 frontleft = modules[0]
 frontRight = modules[1]
 rearLeft = modules[2]
 rearRight = modules[3]
 */

  // The gyro sensor
  private final AHRS gyro = new AHRS();

  // Slew rate filter variables for controlling lateral acceleration
  private double currentRotation = 0.0;
  private double currentTranslationDir = 0.0;
  private double currentTranslationMag = 0.0;

  private SlewRateLimiter magLimiter = new SlewRateLimiter(DrivetrainConstants.DriveConstants.MAG_SLEW_RATE);
  private SlewRateLimiter rotLimiter = new SlewRateLimiter(DrivetrainConstants.DriveConstants.ROT_SLEW_RATE);
  private double prevTime = WPIUtilJNI.now() * 1e-6;
  private double speedFactor = 0.3; // change back to 1.0

  // Odometry class for tracking robot pose
  SwerveDriveOdometry odometry = new SwerveDriveOdometry(
      DrivetrainConstants.DriveConstants.DRIVE_KINEMATICS,
      Rotation2d.fromDegrees(gyro.getAngle()),
      new SwerveModulePosition[] {
          modules[0].getPosition(),
          modules[1].getPosition(),
          modules[2].getPosition(),
          modules[3].getPosition()
      });

  /** Creates a new DriveSubsystem. */
  public RevisedDrivetrain() {
  }

  @Override
  public void periodic() {
    // updates periodically
    odometry.update(
        Rotation2d.fromDegrees(gyro.getAngle()),
        new SwerveModulePosition[] {
          modules[0].getPosition(),
          modules[1].getPosition(),
          modules[2].getPosition(),
          modules[3].getPosition()
        });
    SmartDashboard.putNumber("gyro angle", gyro.getAngle());
    System.out.println("yaw reading" + gyro.getYaw());
    System.out.println("angle reading " + gyro.getAngle());
    // SmartDashboard.putNumber("gyro x", gyroX()); <--
  }

    public void getJoystickValue(CommandXboxController joystick){
    System.out.println("current value: " + joystick.getRightY());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */

  //returns currently-estimated pose of robot
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  // resets the odometry to the specified pose
  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(
        Rotation2d.fromDegrees(gyro.getYaw()),
        new SwerveModulePosition[] {
          modules[0].getPosition(),
          modules[1].getPosition(),
          modules[2].getPosition(),
          modules[3].getPosition()
        },
        pose);
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

    System.out.println("yaw axis: " + getAngle());
    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral acceleration
      double directionSlewRate;

      if (currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(DrivetrainConstants.DriveConstants.DIR_SLEW_RATE / currentTranslationMag);
      } 
      
      else {
        directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
      }
      
      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, currentTranslationDir);

      if (angleDif < 0.45*Math.PI) {
        currentTranslationDir = SwerveUtils.StepTowardsCircular(currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        currentTranslationMag = magLimiter.calculate(inputTranslationMag);
      }

      else if (angleDif > 0.85*Math.PI) {
        if (currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          currentTranslationMag = magLimiter.calculate(0.0);
        }

        else {
          currentTranslationDir = SwerveUtils.WrapAngle(currentTranslationDir + Math.PI);
          currentTranslationMag = magLimiter.calculate(inputTranslationMag);
        }
      }

      else {
        currentTranslationDir = SwerveUtils.StepTowardsCircular(currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
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
    // double xSpeedDelivered = xSpeedCommanded * DriveConstants.MAX_SPEED;
    // double ySpeedDelivered = ySpeedCommanded * DriveConstants.MAX_SPEED;
      double xSpeedDelivered = xSpeedCommanded * DrivetrainConstants.DriveConstants.MAX_SPEED * speedFactor;
      double ySpeedDelivered = ySpeedCommanded * DrivetrainConstants.DriveConstants.MAX_SPEED * speedFactor;

    // SmartDashboard.putNumber("xspeed drive", xSpeedDelivered);
    // SmartDashboard.putNumber("xspeed drive", xSpeedDelivered);
    //System.out.println("xspeed drive: " + xSpeedDelivered);
    //System.out.println("yspeed drive: " + ySpeedDelivered);

    double rotDelivered = currentRotation * DrivetrainConstants.DriveConstants.MAX_ANGULAR_SPEED * 0.75; // get rid of 0.75
    var swerveModuleStates = DrivetrainConstants.DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(getAngle()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DrivetrainConstants.DriveConstants.MAX_SPEED);

    modules[0].setDesiredState(swerveModuleStates[0]); //frontLeft
    modules[1].setDesiredState(swerveModuleStates[1]); // frontRight
    modules[2].setDesiredState(swerveModuleStates[2]); // rearLeft
    modules[3].setDesiredState(swerveModuleStates[3]); // rearRight

  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  // x formation with wheels -> prevent movement
  public void setX() {
    modules[0].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    modules[1].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    modules[2].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    modules[3].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DrivetrainConstants.DriveConstants.MAX_SPEED);
    modules[0].setDesiredState(desiredStates[0]);
    modules[1].setDesiredState(desiredStates[1]);
    modules[2].setDesiredState(desiredStates[2]);
    modules[3].setDesiredState(desiredStates[3]);
  }

  public void resetGyro() {
    gyro.reset();
  }
  public void calibrateGyro(){
    gyro.reset();
  }

  public double getYawAxis(){
    return gyro.getYaw();
  }

  public double setYawOffset() {
    gyro.setAngleAdjustment(-90);
    return gyro.getYaw();
  }

  public double getAngle(){
    //return gyro.getYaw();
    return -1 * gyro.getAngle();
  }

  public double getPitch(){
    return gyro.getPitch();
  }


  public void slowSpeed(){
    speedFactor = 0.5;
  }

  public void regSpeed(){
    speedFactor = 1;
  }

  // resets drive encoders
  public void resetEncoders() {
    modules[0].resetEncoders();
    modules[1].resetEncoders();
    modules[2].resetEncoders();
    modules[3].resetEncoders();
    //Arrays.stream(swerveModules).forEach(RevSwerveModule::resetEncoders);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  // returns heading in degrees (-180 to 180)
  public double getHeading() {
    return Rotation2d.fromDegrees(gyro.getYaw()).getDegrees();
  }



  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  // returns turn rate (deg/s)
  public double getTurnRate() {
    return gyro.getRate() * (DrivetrainConstants.DriveConstants.GYRO_REVERSED ? -1.0 : 1.0);
  }
}
