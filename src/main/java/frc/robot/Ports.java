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
    

    public static final class ButtonPorts {

    }

    public static final class DrivetrainPorts {

    }

    public static final class IntakePorts {

    }

    public static final class HopperPorts {
        public static final int BEAM_BREAK_INTAKE_PORT = 0;
        public static final int BEAM_BREAK_SHOOTER_PORT = 0;

        public static final int HOPPER_MOTOR_PORT = 0;
    }

    public static final class ShooterPorts {
        public static final int LEFT_SHOOTER_MOTOR_PORT = 0;
        public static final int RIGHT_SHOOTER_MOTOR_PORT = 0;

        public static final int LEFT_SHOOTER_FLEX_PORT = 0;
        public static final int RIGHT_SHOOTER_FLEX_PORT = 0;

        public static final int SHOOTER_ANGLE_PORT = 0;

    }

    public static final class ClimbPorts {
        public static final int armRightMotor = 0;
        public static final int armLeftMotor = 1;
        public static final int extendClimb = 1;
        public static final int retractClimb = 0;

        public static final int TOP_SWITCH_PORT = 0;
        public static final int BOTTOM_SWITCH_PORT = 1;
    }

}
