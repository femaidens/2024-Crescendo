// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.photonvision.PhotonCamera;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */

  NetworkTable table = NetworkTableInstance.getDefault().getTable("Limelight");
  NetworkTableEntry tv = table.getEntry("tv");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ta = table.getEntry("ta");
  NetworkTableEntry ts = table.getEntry("ts");
  NetworkTableEntry ty = table.getEntry("ty");

    // Constants such as camera and target height stored. Change per robot and goal!
  public final double cameraHeight;
  public final double targetHeight;
  // Angle between horizontal and the camera.
  public final double cameraPitchRadians;
  // How far from the target we want to be
  public final double goalRange;

  PhotonCamera camera = new PhotonCamera("photonvision");
  
  public Limelight() {
    cameraHeight = Units.inchesToMeters(24);
    targetHeight = Units.feetToMeters(5);
    cameraPitchRadians = Units.degreesToRadians(0);
    goalRange = Units.feetToMeters(3);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    var result = camera.getLatestResult();
  }
}
