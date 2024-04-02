// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Drivetrain;

public class Vision extends SubsystemBase {
  
  private Pose2d botPose;
  private Pose2d estimatePose;
  private LimelightHelpers.LimelightResults jsonDump;
  // private final SwerveDrivePoseEstimator poseEstimator;
  private final Field2d m_field;


  public Vision(){
    botPose = new Pose2d();
    estimatePose = new Pose2d();
    m_field = new Field2d();
  }

  @Override
  public void periodic() {
    updateOdometry();
  }

  public void updateOdometry(){
    jsonDump = LimelightHelpers.getLatestResults("");
    estimatePose = LimelightHelpers.getBotPose2d_wpiBlue(""); //megatag with blue driver


    // LimelightHelpers.Results results = 
    //     LimelightHelpers.getLatestResults("").targetingResults;

    // if(results.valid){
    //   Pose2d camPose = LimelightHelpers.toPose2D(results.botpose_wpiblue);
    //   poseEstimator.addVisionMeasurement(camPose, 
    //   Timer.getFPGATimestamp());
    //   m_field.getObject("Cam est Pose").setPose(camPose);
    // } else {
    //   m_field.getObject("Cam est Pose").setPose(poseEstimator.getEstimatedPosition());
    // }

    // m_field.setRobotPose(poseEstimator.getEstimatedPosition());
  }

  public boolean visionAccurate(Pose2d current){
    return LimelightHelpers.getTV(""); //change
  }
}
