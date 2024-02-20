// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import frc.robot.Ports.*;
import frc.robot.subsystems.ShooterWheel;
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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private CommandXboxController driveJoy = new CommandXboxController(Ports.JoystickPorts.DRIVE_JOY);
  private CommandXboxController operJoy = new CommandXboxController(Ports.JoystickPorts.OPER_JOY);

  private final Intake intake = new Intake();
  private final ShooterWheel shooterWheel = new ShooterWheel();
  private final ShooterAngle shooterAngle = new ShooterAngle();

  private final SendableChooser<Command> autonChooser = new SendableChooser<>();

  public RobotContainer() {
    // configurations
    configureButtonBindings();
    configureAuton();

    // configure default commands
    shooterAngle.setDefaultCommand(
        new RunCommand(
            () -> shooterAngle.setAngle(
                MathUtil.applyDeadband(operJoy.getLeftY(), 0.1)),
            shooterAngle));

    shooterWheel.setDefaultCommand(
        new RunCommand(
            () -> shooterWheel.stopShooter(), shooterWheel));
  }

  public void configureAuton() {
    SmartDashboard.putData("Choose Auto: ", autonChooser);
    // autonChooser.addOption("Angle 60 and shoot", new SpinShooterUp(shooterWheel,
    // shooterWheelAngle));
    // autonChooser.addOption("p1", new Path1(drivetrain, intake, armAngle,
    // armLateral));
    // autonChooser.addOption("p2", new Path2(drivetrain));
    // autonChooser.addOption("test auton", new TestAuton1(drivetrain, intake,
    // armAngle, armLateral));
  }

  private void configureButtonBindings() {
    // resets robot heading (gyro)

    /*
     * // figure out better/more efficient way of creating/binding these cmds to
     * buttons
     * 
     * final Trigger highConeButton = new JoystickButton(operJoy,
     * Ports.XboxControllerMap.Button.Y);
     * highConeButton.onTrue(Commands.sequence(
     * new SetArmAngle(armAngle, PositionConfig.highConeAngle)));
     * // new SetArmExtension(armLateral, PositionConfig.highConeExtend),
     * // new SetClawAngle(intake, IntakeConstants.clawAngle)));
     * 
     */

    Trigger RunRollerButton = operJoy.a(); // change buttons later
    RunRollerButton
        .whileTrue(new RunCommand(
            () -> intake.setIntakeSpeed(Constants.IntakeConstants.ROLLER_SPEED), intake)); 
            // need to code for when it is false

    /*
     * Trigger LiftIntake = operJoy.a(); //change buttons later
     * LiftIntake
     * .whileTrue(new RunCommand(
     * () -> intake.liftIntake(), intake));
     */
    // Trigger shooterSpin = operJoy.a();
    // shooterSpin
    //     .onTrue(new RunCommand(
    //         () -> shooterWheel.setDesiredVelocity(ShooterWheelConstants.SHOOTER_RAD_SECOND), shooterWheel));
    //     // .onFalse(new RunCommand(
    //     //     () -> shooterWheel.stopShooter(), shooterWheel));

    // Trigger shooterUp = operJoy.b();
    // shooterUp
    //     .onTrue(new RunCommand(
    //         () -> shooterAngle.shooterAngleUp(), shooterAngle))
    //     .onFalse(new RunCommand(
    //         () -> shooterAngle.stopShooterAngle(), shooterAngle));

    // 01/23/2024 stacky is sick
    Trigger leftQuasForward = operJoy.rightBumper();
    leftQuasForward
        .whileTrue(
          shooterWheel.leftQuas(SysIdRoutine.Direction.kForward)
        );

    Trigger leftQuasReverse = operJoy.leftBumper();
    leftQuasReverse
        .whileTrue(
          shooterWheel.leftQuas(SysIdRoutine.Direction.kReverse)
        );

    Trigger leftDynaForward = operJoy.rightTrigger();
    leftDynaForward
        .whileTrue(
          shooterWheel.leftDyna(SysIdRoutine.Direction.kForward)
        );

    Trigger leftDynaReverse = operJoy.leftTrigger();
    leftDynaReverse
        .whileTrue(
          shooterWheel.leftDyna(SysIdRoutine.Direction.kReverse)
        );

    Trigger rightQuasForward = operJoy.a();
    rightQuasForward
        .whileTrue(
          shooterWheel.rightQuas(SysIdRoutine.Direction.kForward)
        );
    Trigger rightQuasReverse = operJoy.b();
    rightQuasReverse
        .whileTrue(
          shooterWheel.rightQuas(SysIdRoutine.Direction.kReverse)
        );
    Trigger rightDynaForward = operJoy.x();
    rightDynaForward
        .whileTrue(
          shooterWheel.rightDyna(SysIdRoutine.Direction.kReverse)
        );
    Trigger rightDynaReverse = operJoy.y();
    rightDynaReverse
        .whileTrue(
          shooterWheel.rightDyna(SysIdRoutine.Direction.kReverse)
        );
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