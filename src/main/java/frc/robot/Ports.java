// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class Ports {

    public static final class DrivetrainPorts {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rathker the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(26.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(26.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double FL_CHASSIS_ANGULAR_OFFSET = -Math.PI / 2;
    public static final double FR_CHASSIS_ANGULAR_OFFSET = 0;
    public static final double RL_CHASSIS_ANGULAR_OFFSET = Math.PI;
    public static final double RR_CHASSIS_ANGULAR_OFFSET = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int FRONT_LEFT_DRIVE = 11;
    public static final int REAR_LEFT_DRIVE = 13;
    public static final int FRONT_RIGHT_DRIVE = 15;
    public static final int REAR_RIGHT_DRIVE = 17;

    public static final int FRONT_LEFT_TURNING = 10;
    public static final int REAR_LEFT_TURNING = 12;
    public static final int FRONT_RIGHT_TURNING = 14;
    public static final int REAR_RIGHT_TURNING = 16;

    public static final boolean kGyroReversed = false;
  }

}
