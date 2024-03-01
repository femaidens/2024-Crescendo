// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.*;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterAngleConstants;
import frc.robot.Constants.ShooterWheelConstants;
import frc.robot.Ports.*;
import frc.robot.commands.*;
import frc.robot.subsystems.BeamBreak;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ShooterAngle;
import frc.robot.subsystems.ShooterWheel;
import org.littletonrobotics.urcl.URCL;

public class RobotContainer {

  private CommandXboxController driveJoy = new CommandXboxController(
    Ports.JoystickPorts.DRIVE_JOY
  );
  private CommandXboxController operJoy = new CommandXboxController(
    Ports.JoystickPorts.OPER_JOY
  );

  private final Drivetrain drivetrain = new Drivetrain();
  private final Intake intake = new Intake();
  private final ShooterWheel shooterWheel = new ShooterWheel();
  private final ShooterAngle shooterAngle = new ShooterAngle();
  private final Climb climb = new Climb();
  private final BeamBreak beambreak = new BeamBreak();

  private final SendableChooser<Command> autonChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */

  public RobotContainer() {
    // configurations
    configureButtonBindings();
    configureAuton();

    // configure default commands
    drivetrain.setDefaultCommand(
      // clariy turning with right or with left
      new RunCommand(
        () ->
          drivetrain.drive( // all joy.get values were prev negative
            MathUtil.applyDeadband(-driveJoy.getRightY(), 0.1),
            MathUtil.applyDeadband(-driveJoy.getRightX(), 0.1),
            MathUtil.applyDeadband(-driveJoy.getLeftX(), 0.1),
            true,
            true
          ),
        drivetrain
      )
    ); // field rel = true

    shooterAngle.setDefaultCommand(
      new RunCommand(
        () ->
          shooterAngle.setManualAngle(
            MathUtil.applyDeadband(-operJoy.getRightY(), 0.1)
          ), // CHECK TO SEE IF WE NEED TO NEGATVE INPUT
        shooterAngle
      )
    );

    shooterWheel.setDefaultCommand(
      new RunCommand(() -> shooterWheel.setVelocity(), shooterWheel)
    );
    // beambreak.setDefaultCommand(
    //     new InstantCommand(
    //         () -> beambreak.setEmitter(true), beambreak));
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
    /* BEAM BREAK */
    //     Trigger breaks = operJoy.back();
    //     breaks
    //       .onTrue(new InstantCommand(() -> beambreak.setEmitter(true), beambreak))
    //       .onFalse(
    //         new InstantCommand(() -> beambreak.setEmitter(false), beambreak)
    //       );

    /* CLIMB BUTTONS */

    /* INTAKE BUTTONS */
    Trigger runIntake = operJoy.rightBumper(); // change buttons later
    runIntake
      .onTrue(
        new RunCommand(
          () -> intake.setIntakeSpeed(IntakeConstants.ROLLER_SPEED),
          intake
        )
      )
      .onFalse(new RunCommand(() -> intake.stopIntakeMotor(), intake));

    Trigger runOuttake = operJoy.leftBumper(); // change buttons later
    runOuttake
      .onTrue(
        new RunCommand(
          () -> intake.setIntakeSpeed(-IntakeConstants.ROLLER_SPEED),
          intake
        )
      )
      .onFalse(new RunCommand(() -> intake.stopIntakeMotor(), intake));

    Trigger runHopper = operJoy.start(); // change buttons later
    runHopper
      .onTrue(new RunCommand(() -> intake.setHopperSpeed(0.7), intake)) // need to code for when it is
      .onFalse(new InstantCommand(() -> intake.stopHopperMotor(), intake));

    // positive speed is outwards
    Trigger runShooter = operJoy.rightTrigger();
    runShooter
      .onTrue(
        new RunCommand(() -> shooterWheel.setShooterSpeed(0.5), shooterWheel)
      )
      .onFalse(
        new InstantCommand(() -> shooterWheel.stopShooter(), shooterWheel)
      );

    Trigger runShooterIntake = operJoy.leftTrigger();
    runShooterIntake
      .onTrue(
        new RunCommand(() -> shooterWheel.setShooterSpeed(-0.5), shooterWheel)
      )
      .onFalse(
        new InstantCommand(() -> shooterWheel.stopShooter(), shooterWheel)
      );
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    Trigger extendClimbButton = operJoy.povUp();
    extendClimbButton
      .onTrue(new RunCommand(() -> climb.extendClimbArm(), climb))
      .onFalse(new InstantCommand(() -> climb.stopClimb(), climb));

    Trigger retractClimbButton = operJoy.povDown();
    retractClimbButton
      .onTrue(new RunCommand(() -> climb.retractClimbArm(), climb))
      .onFalse(new InstantCommand(() -> climb.stopClimb(), climb));
    /* HOPPER BUTTONS */

    /* SHOOTER BUTTONS */
    Trigger zerorot = operJoy.a();
    zerorot 
      .whileTrue(new InstantCommand(
        () -> shooterWheel.setVelocitySetpoint(0), shooterWheel));

    Trigger tworot = operJoy.b();
    tworot 
      .whileTrue(new RunCommand(
        () -> shooterWheel.setVelocitySetpoint(2.0 * 360.0), shooterWheel));

    Trigger fiverot = operJoy.x();
    fiverot 
      .whileTrue(new RunCommand(
        () -> shooterWheel.setVelocitySetpoint(5.0 * 360.0), shooterWheel));

    Trigger tenrot = operJoy.y();
    tenrot 
      .whileTrue(new RunCommand(
        () -> shooterWheel.setVelocitySetpoint(10.0 * 360.0), shooterWheel));
    // Trigger ampFlushButton = operJoy.a();
    // ampFlushButton
    // .onTrue(Commands.parallel(
    // shooterAngle.SetShooterAngle(ShooterAngleConstants.AMP_FLUSH),
    // shooterWheel.SetShooterSpeed(ShooterWheelConstants.AMP_FLUSH)))

    // .onFalse(new RunCommand(
    // () -> shooterAngle.setAngle(), shooterAngle));

    // Trigger speakerFlushButton = operJoy.x();
    // speakerFlushButton
    // .onTrue(Commands.parallel(
    // shooterAngle.SetShooterAngle(ShooterAngleConstants.SPEAKER_FLUSH),
    // shooterWheel.SetShooterSpeed(ShooterWheelConstants.SPEAKER_FLUSH)))

    // .onFalse(new RunCommand(
    // () -> shooterAngle.setAngle(), shooterAngle));

    // Trigger speakerStageButton = operJoy.y();
    // speakerStageButton
    // .onTrue(Commands.parallel(
    // shooterAngle.SetShooterAngle(ShooterAngleConstants.SPEAKER_STAGE),
    // shooterWheel.SetShooterSpeed(ShooterWheelConstants.SPEAKER_STAGE)))

    // .onFalse(new RunCommand(
    // () -> shooterAngle.setAngle(), shooterAngle));

    // Trigger speakerWingButton = operJoy.b();
    // speakerWingButton
    // .onTrue(shooterAngle.SetShooterAngle(Constants.ShooterAngleConstants.AMP_FLUSH));

    /* DRIVETRAIN SYSID BUTTONS */
    // Trigger driveForwardQuasistaticButton = driveJoy.leftBumper();
    // driveForwardQuasistaticButton.whileTrue(
    // drivetrain.driveQuasistatic(SysIdRoutine.Direction.kForward));

    // Trigger driveReverseQuasistatic = driveJoy.rightBumper();
    // driveReverseQuasistatic.whileTrue(
    // drivetrain.driveQuasistatic(SysIdRoutine.Direction.kReverse));

    // Trigger driveForwardDynamicButton = driveJoy.leftTrigger();
    // driveForwardDynamicButton.whileTrue(
    // drivetrain.driveDynamic(SysIdRoutine.Direction.kForward));

    // Trigger driveReverseDynamicButton = driveJoy.rightTrigger();
    // driveReverseDynamicButton.whileTrue(
    // drivetrain.driveDynamic(SysIdRoutine.Direction.kReverse));

    // Trigger turnQuasistaticButton = driveJoy.a();
    // turnQuasistaticButton.whileTrue(
    // drivetrain.turnQuasistatic(SysIdRoutine.Direction.kForward));

    // Trigger turnDynamicButton = driveJoy.y();
    // turnDynamicButton.whileTrue(
    // drivetrain.turnDynamic(SysIdRoutine.Direction.kForward));
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
