// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class Vision extends SubsystemBase {
  
  private Pose2d botPose;
  private Pose2d estimatePose;
  private LimelightHelpers.LimelightResults jsonDump;

  public Vision(){
    botPose = new Pose2d();
    estimatePose = new Pose2d();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void updateOdometry(){
    jsonDump = LimelightHelpers.getLatestResults("");
    estimatePose = LimelightHelpers.getBotPose2d_wpiBlue("");
    
  }
}
