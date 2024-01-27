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
  CommandXboxController operJoy = new CommandXboxController(Ports.JoystickPorts.OPER_JOY);
  CommandXboxController driveJoy = new CommandXboxController(Ports.JoystickPorts.DRIVE_JOY);
  private final Intake intake = new Intake();
  // private final Joystick lateralJoy = new Joystick(Ports.JoystickPorts.LATERAL_JOY);
  // private final Joystick rotationJoy = new Joystick(Ports.JoystickPorts.ROTATION_JOY);
  private final SendableChooser<Command> autonChooser = new SendableChooser<>();
  private final Shooter m_shooter = new Shooter();
  private final ShooterAngle m_shooterAngle = new ShooterAngle();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  // private final CommandXboxController driveJoy =
  //     new CommandXboxController(JoystickPorts.OPER_JOY);
  private CommandXboxController m_driverController = new CommandXboxController(Ports.JoystickPorts.DRIVE_JOY);
  private CommandXboxController m_operController = new CommandXboxController(Ports.JoystickPorts.OPER_JOY);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // auton config
    configureAuton();

    // Configure default commands
    m_shooterAngle.setDefaultCommand(
      new RunCommand(
        () -> m_shooterAngle.setShooterAngle(
          MathUtil.applyDeadband(m_operController.getLeftY(), 0.1)),
          m_shooterAngle)
    );

    m_shooter.setDefaultCommand(
      new RunCommand(
        () -> m_shooter.stopShooter(), m_shooter)
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
    // resets robot heading (gyro)

    /*
    // figure out better/more efficient way of creating/binding these cmds to buttons
    
    final Trigger highConeButton = new JoystickButton(operJoy, Ports.XboxControllerMap.Button.Y);
    highConeButton.onTrue(Commands.sequence(
      new SetArmAngle(armAngle, PositionConfig.highConeAngle)));
      // new SetArmExtension(armLateral, PositionConfig.highConeExtend), 
      // new SetClawAngle(intake, IntakeConstants.clawAngle)));

    */

    Trigger RunRollerButton = operJoy.a(); //change buttons later
    RunRollerButton
      .whileTrue(new RunCommand(
        () -> intake.setRollerSpeed(Constants.IntakeConstants.rollerSpeed), intake)); //need to code for when it is false

   /*Trigger LiftIntake = operJoy.a(); //change buttons later
    LiftIntake
      .whileTrue(new RunCommand(
        () -> intake.liftIntake(), intake));*/
  Trigger shooterSpin = m_operController.a();
      shooterSpin
        .onTrue(new RunCommand(
          () -> m_shooter.setDesiredVelocity(ShooterConstants.SHOOTER_METERS_SECOND), m_shooter))
        .onFalse(new RunCommand(
          () -> m_shooter.stopShooter(), m_shooter));

    Trigger shooterUp = m_operController.b();
      shooterUp
        .onTrue(new RunCommand(
          () -> m_shooterAngle.shooterAngleUp(), m_shooterAngle
        ))
        .onFalse(new RunCommand(
          () -> m_shooterAngle.stopShooterAngle(), m_shooterAngle));

    //01/23/2024 stacky is sick 
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