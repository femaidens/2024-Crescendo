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

  }

  public static final class IntakePorts {
    public static final int INTAKE_ROLLER = 3;
    public static final int INTAKE_ENCODER = 4;
  }

  public static final class HopperPorts {
    public static final int RECEIVER_PORT = 0;
    public static final int EMITTER_PORT = 0;

    public static final int HOPPER_MOTOR = 0;
  }

  public static final class ShooterPorts {
    public static final int LEFT_SHOOTER = 0;
    public static final int RIGHT_SHOOTER = 0;

    public static final int LEFT_SHOOTER_FLEX = 0;
    public static final int RIGHT_SHOOTER_FLEX = 0;

    public static final int SHOOTER_ANGLE = 0;
  }

  public static final class ClimbPorts {
    public static final int RIGHT_ARM_MOTOR = 0;
    public static final int LEFT_ARM_MOTOR = 1;
  }

}
