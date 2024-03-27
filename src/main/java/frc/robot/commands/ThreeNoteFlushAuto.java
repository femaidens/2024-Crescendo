// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.IntakeHopperConstants;
import frc.robot.DrivetrainConstants.DriveConstants;
import frc.robot.DrivetrainConstants.ModuleConstants;
import frc.robot.DrivetrainConstants.ModuleConstants.Drive;
import frc.robot.DrivetrainConstants.ModuleConstants.Turning;
import frc.robot.subsystems.Drivetrain;
// import frc.robot.subsystems.Hopper;
// import frc.robot.subsystems.Intake;
// import frc.robot.subsystems.ShooterAngle;
// import frc.robot.subsystems.ShooterWheel;

import java.util.List;

// import java.util.HashMap;
// import java.util.List;

// import com.pathplanner.*;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
// import com.pathplanner.lib.commands.PathPlannerAuto;
// import com.pathplanner.lib.path.PathConstraints;
// import com.pathplanner.lib.path.PathPlannerPath;
// import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
/** Add your docs here. */
public class ThreeNoteFlushAuto extends SequentialCommandGroup{

    public ThreeNoteFlushAuto(Drivetrain drivetrain, Intaking intaking, Shooter shooter){
        List<PathPlannerPath> pathGroup = PathPlannerAuto.getPathGroupFromAutoFile("3 note flush");
        // Pose2d startingPose = PathPlannerAuto.getStaringPoseFromAutoFile("Example Auto");        
        // AutoBuilder autoBuilder = new AutoBuilder();
        AutoBuilder.configureHolonomic(
        drivetrain::getPose, 
        drivetrain::resetOdometry, 
        drivetrain::getRobotRelativeChassisSpeeds, //chassis speed supplier must be robot relative
        drivetrain::setChassisSpeeds, //method that will drive the robot based on robot relative chassis speed
        new HolonomicPathFollowerConfig(
            new PIDConstants(Drive.kP, Drive.kI, Drive.kD), // Translation PID constants
            new PIDConstants(Turning.kP, Turning.kI, Turning.kD), // Rotation PID constants
            DriveConstants.MAX_SPEED, // Max module speed, in m/s
            ModuleConstants.WHEEL_DIAMETER/2, 
            new ReplanningConfig()), 
        () -> {
        var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
        drivetrain);

        addCommands(
            AutoBuilder.buildAuto(pathGroup)) 
        );

    }

}
