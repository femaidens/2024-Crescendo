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

  public static final class IntakeConstants {

  }

  public static final class HopperConstants {

  }

  public static final class ShooterConstants {

  }

  public static final class ClimbConstants {

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
