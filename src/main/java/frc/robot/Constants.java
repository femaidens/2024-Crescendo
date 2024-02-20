// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {

  public static final class OIConstants {
    public static final double kDriveDeadband = 0.05;
  }

  public static final class ClimberConstants {
    public static final double ARM_SPEED = 0.3;
  }

  public static final class HopperConstants {
    public static final double VEL_CFACTOR = 360.0 / 60.0; // 360 degrees/sec
    public static final int CURRENT_LIMIT = 0;
    public static final double TRANSITION_SPEED = 0.05;
  }

  public static final class IntakeConstants {
    // (1/GR) * (1 rot/ min) * (360 degrees/rot) * (1 min/ 60 sec)
    public static final double VEL_CFACTOR = 360.0 / 60.0; // 360 degrees/sec
    public static final double ROLLER_SPEED = 0.5;

    public static final int CURRENT_LIMIT = 0;

    // pid constants
    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    // ff constants
    public static final double kS = 0.0;
    public static final double kV = 0.0;
    public static final double kA = 0.0;
  }

  public static final class ShooterWheelConstants {
    // CONVERSION (RPM -> DEG/S)
    // RPM * 1/(GR) * (360 DEG/ROT) * (1 MIN/60 SEC)

    public static final double VEL_CFACTOR = 360.0 / 60.0;

    // limits
    public static final int CURRENT_LIMIT = 30;

    // auton
    public static final double SHOOTER_METERS_SECOND = 2.0;
    public static final double AUTON_SPEED = 720; // degrees/sec

    // ff
    public static final double kS = 0;
    public static final double kV = 0;

    // pid
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;

  }

  public static final class ShooterAngleConstants {

    // conversion
    public static final double POS_CFACTOR = 360.0; // degrees

    // limits
    public static final int CURRENT_LIMIT = 30;

    // angles (degrees)
    public static final double PHYSICAL_OFFSET = 18.3;
    public static final double SHOOTER_MAX_ANGLE = 75.0;
    public static final double SHOOTER_MIN_ANGLE = PHYSICAL_OFFSET; 

    // auton
    public static final double SHOOTER_ANGLE_UP = 60; // change in shooter later

    // pid
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;

  }
}