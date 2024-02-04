// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }


  public static final class IntakeConstants{
    public static final double rotationSpeed = 0.5;
    public static final double rollerSpeed = 0.5;
  }
    public static final class PIDConstants{
      public static final double kP = 0;
      public static final double kI = 0;
      public static final double kD = 0;
    }

    public static final double rotLiftSetPoint = 5;
    public static final double rotLowerSetPoint = 0;
  public static final class HopperConstants {
    public static final int HOPPER_CURRENT_LIMIT = 0;
  }

  public static final class ShooterConstants {
    //feedforward
    public static final double kS = 0;
    public static final double kV = 0;
    //pid
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    //conversion
    /*
     * to obtain vcf, use dimensional analysis to convert from rpm to m/s
     * given rpm * 1/(gear ratio) * (2 * Pi * radius) * 60 (for seconds)
     */
    public static final double VELOCITY_CONVERSION_FACTOR = 0;
    //limits
    public static final int SHOOTER_CURRENT_LIMIT = 0;

    //auto constants
    public static final double SHOOTER_METERS_SECOND = 2.0;
  }

  public static final class ShooterAngleConstants {
    //pid
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    //conversion
    public static final double POSITION_CONVERSION_FACTOR = 360;
    //limits
    public static final int SHOOTER_ANGLE_CURRENT_LIMIT = 0;
    //autos
    public static final double SHOOTER_ANGLE_UP = 60;
  }

  public static final class DrivetrainConstants {
    // swerve constants have a couple diff classes, so just put all of them at the bottom of the constants class (aka here)
  }

  public static final class ClimberConstants 
  {
    public static final double climbArmSpeed = 0.3;
  }
}