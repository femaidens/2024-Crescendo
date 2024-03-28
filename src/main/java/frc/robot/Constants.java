// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public final class Constants {

  public static final double DEGREES = 360.0;

  public static final class ClimberConstants {
    public static final double ARM_SPEED = 0.85;
  }

  public static final class IntakeHopperConstants {
    public static final double INTAKE_NOTE_SPEED = 4 * DEGREES; // 4, 6 prev, 3 rotations
  }

  public static final class HopperConstants {
    public static final double VEL_CFACTOR = DEGREES / (60.0 * 4.0); // 360 degrees/sec, 4:1 gr
    public static final int CURRENT_LIMIT = 35;
    public static final double TRANSITION_SPEED = 4.5 * DEGREES; //4.5

    public static final double kS = 0.36814; //0.24365; //0.22973; //values as of 3/2
    public static final double kV = 0.0019064; //0.036019; //0.017661; //3/2
    public static final double kA = 0.00012656; //0.0052157; //0.0012902; //3/2
  }

  public static final class IntakeConstants {
    // (1/GR) * (1 rot/ min) * (360 degrees/rot) * (1 min/ 60 sec)
    public static final double VEL_CFACTOR = (DEGREES * 18.0)/ (60.0 * 4.0 * 24.0); // 360 degrees/sec, 4:1 gr, 24:18
    public static final double ROLLER_SPEED = 0.175;

    public static final int CURRENT_LIMIT = 0;

    // velocities
    public static final double INTAKE_VEL =  2*360;// IntakeHopperConstants.INTAKE_NOTE_SPEED; // test it out
    public static final double OUTTAKE_VEL = -INTAKE_VEL;

    // pid constants -> need to populate 
    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    // ff constants -> need to populate
    public static final double kS = 0.32367; //0.32982; //ff values as of 3/2 sysid
    public static final double kV = 0.014425;//0.014425; //0.0014383; //values as of 3/2 sysid
    public static final double kA = 0.00014654; //0.00020542; //values as of 
  }

  public static final class ShooterWheelConstants {
    // CONVERSION (RPM -> DEG/S)
    // RPM * 1/(GR) * (360 DEG/ROT) * (1 MIN/60 SEC)

    public static final double GEAR_RATIO = 24.0 / 18.0; // 24:18; inversed already
    public static final double VEL_CFACTOR = DEGREES / (60.0 * GEAR_RATIO);

    public static final double V_TOLERANCE = 2; // 2 degrees/second

    // limits
    public static final int CURRENT_LIMIT = 30;
    
    public static final double AMP_FLUSH = 13*DEGREES; // placeholder

    public static final double SPEAKER_FLUSH = 57*DEGREES; // placeholder
    public static final double SPEAKER_STAGE = 60*DEGREES;
    // public static final double SPEAKER_WING = 0.0; // placeholder

    // speeds (degrees/sec)
    public static final double DEFAULT_VELOCITY = 5*DEGREES; // check how much voltage this is drawing
    public static final double SHOOTER_INTAKE_SPEED = -DEFAULT_VELOCITY;

    // auton
    public static final double SHOOTER_METERS_SECOND = 2.0;
    public static final double AUTON_SPEED = 2*DEGREES; // degrees/sec

    // ff -> need to populate
    public static final double kS = 0.22054; //0.22083; //as of 2/28 sys id results
    public static final double kV = 0.00047971; //0.00034866; //as of 2/28 sys id results
    public static final double kA = 5.0683E-05; //2.9726E-05;

    // pid -> need to populate
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;

  }

  public static final class ShooterAngleConstants {

    // conversion
    public static final double VEL_CFACTOR = DEGREES / (25 * 60);
    public static final double POS_CFACTOR = DEGREES; // degrees, 25:1 gr

    // limits
    public static final int CURRENT_LIMIT = 30;
    public static final double P_TOLERANCE = 3.0;

    // angles (degrees)
    public static final double PHYSICAL_OFFSET = 18.5; // 18.3
    public static final double MAX_ANGLE = 62.0;
    public static final double MIN_ANGLE = 23.0; 

    //check picture for distances for testing
    public static final double AMP_FLUSH = 62.0; // placeholder acc angle is 58, but set setpoint to be 60-61

    public static final double INITIAL_ANGLE = 65.0;
    public static final double SPEAKER_FLUSH = 60.0; // tested; worked!
    public static final double SPEAKER_STAGE = 38.5; // actual angle -> 36.5
    // public static final double SPEAKER_WING = 25.0; // placeholder
    public static final double DEFAULT_ANGLE = MIN_ANGLE;


    // speeds
    public static final double CONSTANT_SPEED = 0.1;

    // auton

    // pid -> need to populate
    public static final double kP = 0.11;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double kS = 0.1902; //0.13601;
    public static final double kV = 0.0013292; // 0.001622;
    public static final double kA = 0.00083976; //0.00050685;
    public static final double kG = 1.159; // 1.0622;

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

    public static final double TAXI_AMP_TIME = 4; //seconds
    public static final double TAXI_SPEAKER_TIME = 4.0; 
    public static final double DRIVE_TIME = 4.0;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
      AUTON_MAX_ANGULAR_SPEED, AUTON_MAX_ANGULAR_SPEED_SQUARED);

    // shooter wheel speed
    public static final double WHEEL_SPEED = 5.0 * DEGREES;
    // intake wheels
    public static final double AUTON_OUTTAKE_TIME = 2;


  }
}
