// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class DrivetrainConstants {

  public static final class OIConstants {
    public static final double DEADBAND = 0.05;
  }

  public static final class DriveConstants {

    public static final double MAX_SPEED = 4.8; // max speed meters per second *** LOOK INTO MAX ALLOWED SPEED
    public static final double MAX_ANGULAR_SPEED = 2 * Math.PI; // radians per second

    // slew rate constants
    public static final double DIR_SLEW_RATE = 1.2; // radians per second
    public static final double MAG_SLEW_RATE = 1.8; // percent per second (1 = 100%)
    public static final double ROT_SLEW_RATE = 2.0; // percent per second (1 = 100%)

    /* CHASSIS CONFIG */
    // distance between centers of right and left wheels on robot
    public static final double TRACK_WIDTH = Units.inchesToMeters(23.0);

    // distance between front and back wheels on robot -> replace with known values
    public static final double WHEEL_BASE = Units.inchesToMeters(26.5);
    
    public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
        new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2), // fl
        new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2), // fr
        new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2), // rl
        new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2)); // rr

    // angular offsets of the modules relative to the chassis in radians
    public static final double FL_CHASSIS_ANGULAR_OFFSET = -Math.PI / 2;
    public static final double FR_CHASSIS_ANGULAR_OFFSET = 0;
    public static final double RL_CHASSIS_ANGULAR_OFFSET = Math.PI;
    public static final double RR_CHASSIS_ANGULAR_OFFSET = Math.PI / 2;

    public static final boolean GYRO_REVERSED = true; // navx is cw positive
  }

  public static final class ModuleConstants {

    // 14T pinion gears
    public static final int DRIVE_MOTOR_PINION_TEETH = 14;

    // calcs required for driving motor conversion factors and feed forward
    public static final double DRIVE_MOTOR_FREE_SPEED_RPS = NeoMotorConstants.FREE_SPEED_RPM / 60;
    public static final double WHEEL_DIAMETER = 0.0762; // meters
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

    public static final int DRIVE_MOTOR_CURRENT_LIMIT = 35; // amps
    public static final int TURNING_MOTOR_CURRENT_LIMIT = 35; // amps
    
    public static final IdleMode kDriveMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final class Drive {
      // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
      public static final double DRIVE_MOTOR_REDUCTION = (45.0 * 22) / (DRIVE_MOTOR_PINION_TEETH * 15);
      public static final double DRIVE_WHEEL_FREE_SPEED_RPS = (DRIVE_MOTOR_FREE_SPEED_RPS * WHEEL_CIRCUMFERENCE)
          / DRIVE_MOTOR_REDUCTION;

      // drive encoder position factor
      public static final double DRIVE_ENCODER_PFACTOR = (WHEEL_DIAMETER * Math.PI)
          / DRIVE_MOTOR_REDUCTION; // meters

      // velocity factor
      public static final double DRIVE_ENCODER_VFACTOR = ((WHEEL_DIAMETER * Math.PI)
          / DRIVE_MOTOR_REDUCTION) / 60.0; // meters per second

      public static final double kP = 0.03; // initially 0.04
      public static final double kI = 0;
      public static final double kD = 0.01;

      public static final double kS = 0.0; // placeholder --> run sysid
      public static final double kA = 0.0; // placeholder 
      public static final double kV = 0.0; //placeholder
      
      public static final double kFF = 1 / DRIVE_WHEEL_FREE_SPEED_RPS;
      public static final double kMinOutput = -1;
      public static final double kMaxOutput = 1;
    }

    public static final class Turning {
      // Invert the turning encoder, since the output shaft rotates in the opposite direction of
      // the steering motor in the MAXSwerve Module.
      public static final boolean ENCODER_INVERTED = true;
      // turning encoder position factor
      public static final double ENCODER_PFACTOR = (2 * Math.PI); // radians
      // velocity factor
      public static final double ENCODER_VFACTOR = (2 * Math.PI) / 60.0; // radians per second

      // turning encoder position pid min input
      public static final double ENCODER_PPID_MIN = 0; // radians
      // max input
      public static final double ENCODER_PPID_MAX = ENCODER_PFACTOR; // radians

      public static final double kP = 0.2; // initally 1
      public static final double kI = 0;
      public static final double kD = 0.05;
      public static final double kFF = 0; // need to test?

      public static final double kS = 0; // placeholder
      public static final double kA = 0; // placeholder
      public static final double kV = 0; // placeholder

      public static final double kMinOutput = -1;
      public static final double kMaxOutput = 1;
    }
  }

  public static final class NeoMotorConstants {
    public static final double FREE_SPEED_RPM = 5676;
  }
}