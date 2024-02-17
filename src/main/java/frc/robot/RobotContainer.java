// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
  // The robot's subsystems and commands are defined here...
  private final Drivetrain drivetrain = new Drivetrain();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  // private final CommandXboxController driveJoy =
  //     new CommandXboxController(JoystickPorts.OPER_JOY);
  private CommandXboxController driveJoy = new CommandXboxController(Ports.JoystickPorts.DRIVE_JOY);
  private CommandXboxController operJoy = new CommandXboxController(Ports.JoystickPorts.OPER_JOY);
  private final SendableChooser<Command> autonChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // auton config
    configureAuton();

    // DataLogManager.start();
    // URCL.start();

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

    Trigger driveForwardQuasistaticButton = driveJoy.leftBumper();
    driveForwardQuasistaticButton.whileTrue(
      drivetrain.driveQuasistatic(SysIdRoutine.Direction.kForward));

    Trigger driveReverseQuasistatic = driveJoy.rightBumper();
    driveReverseQuasistatic.whileTrue(
      drivetrain.driveQuasistatic(SysIdRoutine.Direction.kReverse));

    Trigger driveForwardDynamicButton = driveJoy.leftTrigger();
    driveForwardDynamicButton.whileTrue(
      drivetrain.driveDynamic(SysIdRoutine.Direction.kForward));

    Trigger driveReverseDynamicButton = driveJoy.rightTrigger();
    driveReverseDynamicButton.whileTrue(
      drivetrain.driveDynamic(SysIdRoutine.Direction.kReverse));

    Trigger turnQuasistaticButton = driveJoy.a();
    turnQuasistaticButton.whileTrue(
      drivetrain.turnQuasistatic(SysIdRoutine.Direction.kForward));

    Trigger turnDynamicButton = driveJoy.y();
    turnDynamicButton.whileTrue(
      drivetrain.turnDynamic(SysIdRoutine.Direction.kForward));

    /*
    // figure out better/more efficient way of creating/binding these cmds to buttons
    
    final Trigger highConeButton = new JoystickButton(operJoy, Ports.XboxControllerMap.Button.Y);
    highConeButton.onTrue(Commands.sequence(
      new SetArmAngle(armAngle, PositionConfig.highConeAngle)));
      // new SetArmExtension(armLateral, PositionConfig.highConeExtend), 
      // new SetClawAngle(intake, IntakeConstants.clawAngle)));

    
    final Trigger resetIntakeButton = new JoystickButton(operJoy, ButtonPorts.RESET_INTAKE_BUTTON_PORT);
    resetIntakeButton.onTrue(
      // Commands.parallel(
      new SetArmAngle(armAngle, ArmConstants.DEFAULT_ARM_ANGLE));
      // new SetArmExtension(armLateral, PositionConfig.defaultExtension), 
      // new SetClawAngle(intake, IntakeConstants.defaultClawAngle)));

    final Trigger floorScoreButton = new JoystickButton(operJoy, ButtonPorts.FLOOR_SCORE_BUTTON_PORT);
    floorScoreButton.onTrue(Commands.sequence(
      new OpenClaw(intake), 
      new SetArmExtension(armLateral, PositionConfig.defaultExtension), 
      new SetClawAngle(intake, IntakeConstants.clawAngle)));

    final Trigger floorIntakeButton = new JoystickButton(operJoy, ButtonPorts.FLOOR_INTAKE_BUTTON_PORT);
    floorIntakeButton.onTrue(Commands.sequence(
      new OpenClaw(intake), 
      new IntakeGP(intake), 
      new CloseClaw(intake)));

    final Trigger humanPlayerButton = new JoystickButton(operJoy, ButtonPorts.HP_BUTTON_PORT);
    humanPlayerButton.onTrue(Commands.sequence(
      new SetArmAngle(armAngle, PositionConfig.highConeAngle))); 
      // new SetArmExtension(armLateral, PositionConfig.midConeExtend), 
      // new SetClawAngle(intake, IntakeConstants.clawAngle)));
    // substation distance (95cm) is similar to mid node distance (90cm)
    */
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