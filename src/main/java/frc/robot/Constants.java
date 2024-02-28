// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public final class Constants {

  public static final class OIConstants {
    public static final double kDriveDeadband = 0.05;
  }

  public static final class ClimberConstants {
    public static final double ARM_SPEED = 0.1;
  }

  public static final class HopperConstants {
    public static final double VEL_CFACTOR = 360.0 / (60.0 * 100.0); // 360 degrees/sec, 100:1 gr
    public static final int CURRENT_LIMIT = 35;
    public static final double TRANSITION_SPEED = 0.05;
  }

  public static final class IntakeConstants {
    // (1/GR) * (1 rot/ min) * (360 degrees/rot) * (1 min/ 60 sec)
    public static final double VEL_CFACTOR = 360.0 / (60.0 * 4.0); // 360 degrees/sec, 4:1 gr
    public static final double ROLLER_SPEED = 0.5;

    public static final int CURRENT_LIMIT = 0;

    // pid constants -> need to populate
    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    // ff constants -> need to populate
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
    
    public static final double AMP_FLUSH = 0.0; // placeholder

    public static final double SPEAKER_FLUSH = 0.0; // placeholder
    public static final double SPEAKER_STAGE = 0.0; // placeholder
    public static final double SPEAKER_WING = 0.0; // placeholder
    // speeds (degrees/sec)

    // auton
    public static final double SHOOTER_METERS_SECOND = 2.0;
    public static final double AUTON_SPEED = 720; // degrees/sec

    // ff -> need to populate
    public static final double kS = 0;
    public static final double kV = 0;
    public static final double kA = 0;

    // pid -> need to populate
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;

  }

  public static final class ShooterAngleConstants {

    // conversion
    public static final double POS_CFACTOR = 360.0; // degrees, 25:1 gr

    // limits
    public static final int CURRENT_LIMIT = 30;

    // angles (degrees)
    public static final double PHYSICAL_OFFSET = 18.5; // 18.3
    public static final double SHOOTER_MAX_ANGLE = 75.0;
    public static final double SHOOTER_MIN_ANGLE = PHYSICAL_OFFSET; 

    //check picture for distances for testing
    public static final double AMP_FLUSH = PHYSICAL_OFFSET; // placeholder

    public static final double SPEAKER_FLUSH = 70.0; // placeholder
    public static final double SPEAKER_STAGE = 40.0; // placeholder
    public static final double SPEAKER_WING = 20.0; // placeholder

    // speeds
    public static final double CONSTANT_SPEED = 0.3;

    // auton
    public static final double SHOOTER_ANGLE_UP = 60; // change in shooter later

    // pid -> need to populate
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;

  }

  public static final class AutoConstants {
    // drivetrain
    public static final double AUTON_MAX_SPEED = 3; // max meters per second
    public static final double AUTON_MAX_ACC = 3; // max acc m/s^2
    public static final double AUTON_MAX_ANGULAR_SPEED = Math.PI; // max angular speed rad/s
    public static final double AUTON_MAX_ANGULAR_SPEED_SQUARED = Math.PI; // angular speed rad/s^2
   
    public static final double PXController = 1;
    public static final double PYController = 1;
    public static final double PThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
      AUTON_MAX_ANGULAR_SPEED, AUTON_MAX_ANGULAR_SPEED_SQUARED);

    // intake wheels
    public static final double AUTON_OUTTAKE_TIME = 2;

    // arm angles
    public static final double AUTON_INC_ARM_ANGLE_TIME = 2.5; // CHANGE AFTER TESTING
    public static final double AUTON_DEC_ARM_ANGLE_TIME = 1.4;

    // taxi time

    // auton drive speeds

  }
}
