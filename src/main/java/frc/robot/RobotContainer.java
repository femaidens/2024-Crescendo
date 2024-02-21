// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Ports.*;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterAngle;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.*;
import frc.robot.commands.*;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // The driver's controller
  private final CommandXboxController operJoy = new CommandXboxController(Ports.JoystickPorts.OPER_JOY);
  private final CommandXboxController driveJoy = new CommandXboxController(Ports.JoystickPorts.DRIVE_JOY);

  private final Intake intake = new Intake();
  private final Climb climb = new Climb();
  private final Shooter shooter = new Shooter();
  private final ShooterAngle shooterAngle = new ShooterAngle();

  private final SendableChooser<Command> autonChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // auton config
    configureAuton();

    // Configure default commands
    shooterAngle.setDefaultCommand(
      new RunCommand(
        () -> shooterAngle.setShooterAngle(
          MathUtil.applyDeadband(operJoy.getLeftY(), 0.1)),
          shooterAngle)
    );

    shooter.setDefaultCommand(
      new RunCommand(
        () -> shooter.stopShooter(), shooter)
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
   */

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

    Trigger RunRollerButton = operJoy.a(); //change buttons later
    RunRollerButton
      .whileTrue(new RunCommand(
        () -> intake.setRollerSpeed(Constants.IntakeConstants.rollerSpeed), intake)); //need to code for when it is false

   /*Trigger LiftIntake = operJoy.a(); //change buttons later
    LiftIntake
      .whileTrue(new RunCommand(
        () -> intake.liftIntake(), intake));*/
  Trigger shooterSpin = operJoy.a();
      shooterSpin
        .onTrue(new RunCommand(
          () -> shooter.setDesiredVelocity(ShooterConstants.SHOOTER_METERS_SECOND), shooter))
        .onFalse(new RunCommand(
          () -> shooter.stopShooter(), shooter));

    Trigger shooterUp = operJoy.b();
      shooterUp
        .onTrue(new RunCommand(
          () -> shooterAngle.shooterAngleUp(), shooterAngle
        ))
        .onFalse(new RunCommand(
          () -> shooterAngle.stopShooterAngle(), shooterAngle));

    //01/23/2024 stacky is sick 
  }

  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    Trigger extendClimbButton = operJoy.b();
    extendClimbButton
      .onTrue(new RunCommand(
        () -> climb.extendClimbArm(), climb))
      .onFalse(new InstantCommand(
        () -> climb.stopClimb(), climb));

    Trigger retractClimbButton = operJoy.a();
    retractClimbButton
      .onTrue(new RunCommand(
        () -> climb.retractClimbArm(), climb))
      .onFalse(new InstantCommand(
        () -> climb.stopClimb(), climb));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    //m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

    //driveJoy.a().whileTrue(climber.extendClimbArm());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    // TrajectoryConfig config = new TrajectoryConfig(
    //     AutoConstants.AUTON_MAX_SPEED,
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