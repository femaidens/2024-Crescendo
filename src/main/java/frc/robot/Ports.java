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
    public static final int DRIVE_JOY = 0;
    public static final int OPER_JOY = 1; // left side of laptop
  }

  public static final class DrivetrainPorts {
    // SPARK MAX CAN IDs
    public static final int FRONT_LEFT_DRIVE = 3;
    public static final int REAR_LEFT_DRIVE = 5; // 4
    public static final int FRONT_RIGHT_DRIVE = 4; // 5
    public static final int REAR_RIGHT_DRIVE = 6;

    public static final int FRONT_LEFT_TURNING = 7;
    public static final int REAR_LEFT_TURNING = 1; // 8
    public static final int FRONT_RIGHT_TURNING = 8; // 1
    public static final int REAR_RIGHT_TURNING = 2;
  }

}
