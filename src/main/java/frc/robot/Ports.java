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
    
  public static final class IntakePorts{
    public static final int rotationNEOPort = 2;
    public static final int rollerNEOPort = 3;
    public static final int encoderPort = 4;
  }

}
