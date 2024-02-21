// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ShooterWheelConstants;
import frc.robot.Ports.*;
import org.littletonrobotics.urcl.URCL;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ShooterWheel;
import frc.robot.subsystems.ShooterAngle;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class RobotContainer {

  private CommandXboxController driveJoy = new CommandXboxController(Ports.JoystickPorts.DRIVE_JOY);
  private CommandXboxController operJoy = new CommandXboxController(Ports.JoystickPorts.OPER_JOY);

  private final Drivetrain drivetrain = new Drivetrain();
  private final Intake intake = new Intake();
  private final ShooterWheel shooterWheel = new ShooterWheel();
  private final ShooterAngle shooterAngle = new ShooterAngle();

  private final SendableChooser<Command> autonChooser = new SendableChooser<>();

  public RobotContainer() {
    // configurations
    configureButtonBindings();
    configureAuton();

    // configure default commands
    drivetrain.setDefaultCommand(
     // clariy turning with right or with left
      new RunCommand(
          () -> drivetrain.drive( // all joy.get values were prev negative
              MathUtil.applyDeadband(-driveJoy.getRightY(), 0.1),
              MathUtil.applyDeadband(-driveJoy.getRightX(), 0.1),
              MathUtil.applyDeadband(-driveJoy.getLeftX(), 0.1),
              true, true),
          drivetrain)); // field rel = true

    shooterAngle.setDefaultCommand(
        new RunCommand(
            () -> shooterAngle.setShooterAngle(
                MathUtil.applyDeadband(operJoy.getLeftY(), 0.1)),
            shooterAngle));

    shooterWheel.setDefaultCommand(
        new RunCommand(
            () -> shooterWheel.stopShooter(), shooterWheel));
  }

  public void configureAuton() {
    SmartDashboard.putData("Choose Auto: ", autonChooser);
    // autonChooser.addOption("Angle 60 and shoot", new SpinShooterUp(m_shooter,
    // m_shooterAngle));
    // autonChooser.addOption("p1", new Path1(drivetrain, intake, armAngle,
    // armLateral));
    // autonChooser.addOption("p2", new Path2(drivetrain));
    // autonChooser.addOption("test auton", new TestAuton1(drivetrain, intake,
    // armAngle, armLateral));
  }

  private void configureButtonBindings() {

    Trigger RunRollerButton = operJoy.a(); // change buttons later
    RunRollerButton
        .whileTrue(new RunCommand(
            () -> intake.setIntakeSpeed(Constants.IntakeConstants.ROLLER_SPEED), intake)); // need to code for when it is
                                                                                          // false

    Trigger shooterSpin = operJoy.a();
    shooterSpin
        .onTrue(new RunCommand(
            () -> shooterWheel.setDesiredVelocity(ShooterWheelConstants.SHOOTER_METERS_SECOND), shooterWheel))
        .onFalse(new RunCommand(
            () -> shooterWheel.stopShooter(), shooterWheel));

    Trigger shooterUp = operJoy.b();
    shooterUp
        .onTrue(new RunCommand(
            () -> shooterAngle.shooterAngleUp(), shooterAngle))
        .onFalse(new RunCommand(
            () -> shooterAngle.stopShooterAngle(), shooterAngle));
            
    /* DRIVETRAIN SYSID BUTTONS */
    // Trigger driveForwardQuasistaticButton = driveJoy.leftBumper();
    // driveForwardQuasistaticButton.whileTrue(
    //   drivetrain.driveQuasistatic(SysIdRoutine.Direction.kForward));

    // Trigger driveReverseQuasistatic = driveJoy.rightBumper();
    // driveReverseQuasistatic.whileTrue(
    //   drivetrain.driveQuasistatic(SysIdRoutine.Direction.kReverse));

    // Trigger driveForwardDynamicButton = driveJoy.leftTrigger();
    // driveForwardDynamicButton.whileTrue(
    //   drivetrain.driveDynamic(SysIdRoutine.Direction.kForward));

    // Trigger driveReverseDynamicButton = driveJoy.rightTrigger();
    // driveReverseDynamicButton.whileTrue(
    //   drivetrain.driveDynamic(SysIdRoutine.Direction.kReverse));

    // Trigger turnQuasistaticButton = driveJoy.a();
    // turnQuasistaticButton.whileTrue(
    //   drivetrain.turnQuasistatic(SysIdRoutine.Direction.kForward));

    // Trigger turnDynamicButton = driveJoy.y();
    // turnDynamicButton.whileTrue(
    //   drivetrain.turnDynamic(SysIdRoutine.Direction.kForward));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    // TrajectoryConfig config = new TrajectoryConfig(
    // AutoConstants.AUTON_MAX_SPEED,
    // AutoConstants.AUTON_MAX_ACC)
    // // Add kinematics to ensure max speed is actually obeyed
    // .setKinematics(DriveConstants.DRIVE_KINEMATICS);
        // AutoConstants.AUTON_MAX_SPEED,
    //     AutoConstants.AUTON_MAX_ACC)
    //     // Add kinematics to ensure max speed is actually obeyed
    //     .setKinematics(DriveConstants.DRIVE_KINEMATICS);

    // // An example trajectory to follow. All units in meters.
    // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
    // // Start at the origin facing the +X direction
    // new Pose2d(0, 0, new Rotation2d(0)),
    // // Pass through these two interior waypoints, making an 's' curve path
    // List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
    // // End 3 meters straight ahead of where we started, facing forward
    // new Pose2d(3, 0, new Rotation2d(0)),
    // config);

    // var thetaController = new ProfiledPIDController(
    // AutoConstants.PThetaController, 0, 0,
    // AutoConstants.kThetaControllerConstraints);
    // thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // SwerveControllerCommand swerveControllerCommand = new
    // SwerveControllerCommand(
    // exampleTrajectory,
    // drivetrain::getPose, // Functional interface to feed supplier
    // DriveConstants.DRIVE_KINEMATICS,

    // // Position controllers
    // new PIDController(AutoConstants.PXController, 0, 0),
    // new PIDController(AutoConstants.PYController, 0, 0),
    // thetaController,
    // drivetrain::setModuleStates,
    // drivetrain);

    // // Reset odometry to the starting pose of the trajectory.
    // drivetrain.resetOdometry(exampleTrajectory.getInitialPose());

    // // Run path following command, then stop at the end.
    // return swerveControllerCommand.andThen(() -> drivetrain.drive(0, 0, 0,
    // false));
    return autonChooser.getSelected();
  }
}