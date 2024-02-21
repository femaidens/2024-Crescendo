// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class Ports {
  public static final class JoystickPorts {
    public static final int OPER_JOY = 0;
    public static final int DRIVE_JOY = 1;
  }

  public static final class DrivetrainPorts {
    public static final int FRONT_LEFT_DRIVE = 3;
    public static final int REAR_LEFT_DRIVE = 5;
    public static final int FRONT_RIGHT_DRIVE = 4;
    public static final int REAR_RIGHT_DRIVE = 6;

    public static final int FRONT_LEFT_TURNING = 7;
    public static final int REAR_LEFT_TURNING = 14;
    public static final int FRONT_RIGHT_TURNING = 8;
    public static final int REAR_RIGHT_TURNING = 2;
  }

  public static final class IntakePorts {
    public static final int INTAKE_ROLLER = 9;
  }

  public static final class HopperPorts {
    public static final int RECEIVER = 1;
    public static final int EMITTER = 2;

    public static final int HOPPER_MOTOR = 13;
  }

  public static final class ShooterPorts {
    public static final int LEFT_SHOOTER = 12;
    public static final int RIGHT_SHOOTER = 16;

    // public static final int LEFT_SHOOTER_FLEX = 0;
    // public static final int RIGHT_SHOOTER_FLEX = 0;

    public static final int SHOOTER_ANGLE = 15;
  }

  public static final class ClimbPorts {
    public static final int RIGHT_ARM_MOTOR = 11;
    public static final int LEFT_ARM_MOTOR = 10;

    public static final int TOP_SWITCH_PORT = 9;
    public static final int BOTTOM_SWITCH_PORT = 3;
  }

}
