// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
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
  private PhotonPipelineResult result;
  PhotonTrackedTarget target = result.getBestTarget();

  // Get information from target.
  double yaw = target.getYaw();
  double pitch = target.getPitch();
  double area = target.getArea();
  double skew = target.getSkew();
  Transform3d pose = target.getBestCameraToTarget();
  List<TargetCorner> corners = target.getDetectedCorners();

  // Get information from target.
  int targetID = target.getFiducialId();
  double poseAmbiguity = target.getPoseAmbiguity();
  Transform3d bestCameraToTarget = target.getBestCameraToTarget();
  Transform3d alternateCameraToTarget = target.getAlternateCameraToTarget();

  AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

  Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), 
  aprilTagFieldLayout.getTagPose(target.getFiducialId()), cameraToRobot);

  cam = new PhotonCamera("testCamera");
  Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), 
  new Rotation3d(0,0,0)); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.

// Construct PhotonPoseEstimator
  PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, cam, robotToCam);

  Rotation2d targetYaw = PhotonUtils.getYawToPose(robotPose, targetPose);

  
  public Limelight() {
    cameraHeight = Units.inchesToMeters(24);
    targetHeight = Units.feetToMeters(5);
    cameraPitchRadians = Units.degreesToRadians(0);
    goalRange = Units.feetToMeters(3);
    result = camera.getLatestResult();
  }

  public void estimateTranslationToTarget() {
    Translation2d translation = PhotonUtils.estimateCameraToTargetTranslation(
    distanceMeters, Rotation2d.fromDegrees(-target.getYaw()));
  }

  public void capturePreProcessImage() {
    camera.takeInputSnapshot();
  }

  public void capturePostProcessImage() {
    camera.takeOutputSnapshot();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    camera.setDriverMode(true);
  }
}