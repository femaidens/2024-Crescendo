// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Ports.*;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
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
    drivetrain.resetGyro();
    drivetrain.resetEncoders();

    // auton config
    configureAuton();

    // Configure default commands
    drivetrain.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> drivetrain.drive( // all joy.get values were prev negative
                MathUtil.applyDeadband(-driveJoy.getRightY(), 0.1),
                MathUtil.applyDeadband(-driveJoy.getRightX(), 0.1),
                MathUtil.applyDeadband(-driveJoy.getLeftX(), 0.1),
                true, true),
            drivetrain)

        // new RunCommand(
        //     () -> drivetrain.drive(
        //         MathUtil.applyDeadband(-lateralJoy.getY(), 0.05),
        //         MathUtil.applyDeadband(-lateralJoy.getX(), 0.05),
        //         MathUtil.applyDeadband(-rotationJoy.getX(), 0.05),
        //         true),
        //     drivetrain)
    );
  }

  public void configureAuton() {
    SmartDashboard.putData("Choose Auto: ", autonChooser);

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
  private void configureButtonBindings() {

    Trigger extendClimbButton = operJoy.a();
    extendClimbButton
      .whileTrue(new InstantCommand(
        () -> drivetrain.setX(), drivetrain));

    Trigger resetGyroButton = operJoy.rightBumper();
    resetGyroButton
      .onTrue(new RunCommand(
        () -> drivetrain.resetGyro(), drivetrain));

    /*
    // figure out better/more efficient way of creating/binding these cmds to buttons
    final Trigger midCubeButton = new JoystickButton(operJoy, Ports.XboxControllerMap.Button.A);
    midCubeButton.onTrue(Commands.sequence(
      new SetArmAngle(armAngle, PositionConfig.midCubeAngle), 
      new SetArmExtension(armLateral, PositionConfig.midCubeExtend), 
      new SetClawAngle(intake, IntakeConstants.clawAngle)));

    final Trigger midConeButton = new JoystickButton(operJoy, Ports.XboxControllerMap.Button.B);
    midConeButton.onTrue(Commands.sequence(
      new SetArmAngle(armAngle, PositionConfig.midConeAngle)));
      // new SetArmExtension(armLateral, PositionConfig.midConeExtend), 
      // new SetClawAngle(intake, IntakeConstants.clawAngle)));

    final Trigger highCubeButton = new JoystickButton(operJoy, Ports.XboxControllerMap.Button.X); //change command for testing angle
    highCubeButton.onTrue(Commands.sequence(
      new SetArmAngle(armAngle, PositionConfig.highCubeAngle)));
      // new SetArmExtension(armLateral, PositionConfig.highCubeExtend), 
      // new SetClawAngle(intake, IntakeConstants.clawAngle)));

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