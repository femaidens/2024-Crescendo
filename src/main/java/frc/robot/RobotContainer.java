// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AutoConstants;
import frc.robot.Ports.*;
import frc.robot.subsystems.Drivetrain;

import org.littletonrobotics.urcl.URCL;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private final Drivetrain drivetrain = new Drivetrain();

  private CommandXboxController driveJoy = new CommandXboxController(Ports.JoystickPorts.DRIVE_JOY);
  private CommandXboxController operJoy = new CommandXboxController(Ports.JoystickPorts.OPER_JOY);

  private final SendableChooser<Command> autonChooser = new SendableChooser<>();

  public RobotContainer() {

    configureButtonBindings();
    configureAuton();

    drivetrain.setDefaultCommand(
      // The left stick controls translation of the robot.
      // Turning is controlled by the X axis of the right stick.
      new RunCommand(
          () -> drivetrain.drive( // all joy.get values were prev negative
              MathUtil.applyDeadband(-driveJoy.getRightY(), 0.1),
              MathUtil.applyDeadband(-driveJoy.getRightX(), 0.1),
              MathUtil.applyDeadband(-driveJoy.getLeftX(), 0.1),
              true, true),
          drivetrain) // change field relative to true!!
    );

  }
    public void configureAuton() {
      SmartDashboard.putData("Choose Auto: ", autonChooser);
      //autonChooser.addOption("Angle 60 and shoot", new SpinShooterUp(m_shooter, m_shooterAngle));
    // autonChooser.addOption("p1", new Path1(drivetrain, intake, armAngle, armLateral));
    // autonChooser.addOption("p2", new Path2(drivetrain));
    // autonChooser.addOption("test auton", new TestAuton1(drivetrain, intake, armAngle, armLateral));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
    // Configure the trigger bindings
    configureBindings();

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureButtonBindings() {

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
    //     AutoConstants.AUTON_MAX_ACC)
    //     // Add kinematics to ensure max speed is actually obeyed
    //     .setKinematics(DriveConstants.DRIVE_KINEMATICS);

    // // An example trajectory to follow. All units in meters.
    // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
    //     // Start at the origin facing the +X direction
    //     new Pose2d(0, 0, new Rotation2d(0)),
    //     // Pass through these two interior waypoints, making an 's' curve path
    //     List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
    //     // End 3 meters straight ahead of where we started, facing forward
    //     new Pose2d(3, 0, new Rotation2d(0)),
    //     config);

    // var thetaController = new ProfiledPIDController(
    //     AutoConstants.PThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    // thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
    //     exampleTrajectory,
    //     drivetrain::getPose, // Functional interface to feed supplier
    //     DriveConstants.DRIVE_KINEMATICS,

    //     // Position controllers
    //     new PIDController(AutoConstants.PXController, 0, 0),
    //     new PIDController(AutoConstants.PYController, 0, 0),
    //     thetaController,
    //     drivetrain::setModuleStates,
    //     drivetrain);

    // // Reset odometry to the starting pose of the trajectory.
    // drivetrain.resetOdometry(exampleTrajectory.getInitialPose());

    // // Run path following command, then stop at the end.
    // return swerveControllerCommand.andThen(() -> drivetrain.drive(0, 0, 0, false));
    return autonChooser.getSelected();
  }
}