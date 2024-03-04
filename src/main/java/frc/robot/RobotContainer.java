// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterAngleConstants;
import frc.robot.Constants.ShooterWheelConstants;
import frc.robot.Ports.*;
import frc.robot.commands.Shooter;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ShooterAngle;
import frc.robot.subsystems.ShooterWheel;

import org.littletonrobotics.urcl.URCL;

public class RobotContainer {

  private CommandXboxController driveJoy = new CommandXboxController(JoystickPorts.DRIVE_JOY);
  private CommandXboxController operJoy = new CommandXboxController(JoystickPorts.OPER_JOY);

  private final Drivetrain drivetrain = new Drivetrain();
  private final Intake intake = new Intake();
  private final Hopper hopper = new Hopper();
  private final ShooterWheel shooterWheel = new ShooterWheel();
  private final ShooterAngle shooterAngle = new ShooterAngle();
  private final Climb climb = new Climb();

  private final Shooter shooter = new Shooter(shooterAngle, shooterWheel, intake);

  private final SendableChooser<Command> autonChooser = new SendableChooser<>();

  public RobotContainer() {
    // configurations
    configureButtonBindings();
    configureAuton();
    configureDefaultCommands();
  }

  public void configureDefaultCommands() {
    drivetrain.setDefaultCommand(
        new RunCommand(
            () -> drivetrain.drive( // all joy.get values were prev negative
                MathUtil.applyDeadband(-driveJoy.getRightY(), 0.1),
                MathUtil.applyDeadband(-driveJoy.getRightX(), 0.1),
                MathUtil.applyDeadband(-driveJoy.getLeftX(), 0.1),
                true,
                true),
            drivetrain)); // field rel = true

    shooterAngle.setDefaultCommand(
        // new RunCommand(
        //     () -> shooterAngle.setManualAngle(
        //         MathUtil.applyDeadband(-operJoy.getRightY(), 0.1)),
        //     shooterAngle));
        new RunCommand(
            () -> shooterAngle.setAngle(), shooterAngle));

    shooterWheel.setDefaultCommand(
        new RunCommand(() -> shooterWheel.setVelocity(), shooterWheel));
    // beambreak.setDefaultCommand(
    // new InstantCommand(
    // () -> beambreak.setEmitter(true), beambreak));
  }
  
  public void configureAuton() {
    SmartDashboard.putData("Choose Auto: ", autonChooser);
    autonChooser.addOption("Shoot Amp", shooter.autonShoot(ShooterAngleConstants.AMP_FLUSH));
  }

  private void configureButtonBindings() {

    /* * * CLIMB BUTTONS * * */
    // extend climb arm
    operJoy.povUp()
        .onTrue(climb.extendClimbCmd())
        .onFalse(climb.stopMotorsCmd());

    // retract climb arm
    operJoy.povDown()
        .onTrue(climb.retractClimbCmd())
        .onFalse(climb.stopMotorsCmd());

    /* * * INTAKE BUTTONS * * */
    // runs intake
    operJoy.rightBumper()
        .onTrue(intake.setIntakeSpeedCmd(IntakeConstants.ROLLER_SPEED))
        .onFalse(intake.stopIntakeMotorCmd());

    // runs outtake
    operJoy.leftBumper()
        .onTrue(intake.setIntakeSpeedCmd(-IntakeConstants.ROLLER_SPEED))
        .onFalse(intake.stopIntakeMotorCmd());

    /* * * HOPPER BUTTONS * * */
    // runs hopper (towards shooter)
    operJoy.start()
        .onTrue(intake.setHopperSpeedCmd(0.7))
        .onFalse(intake.stopHopperMotorCmd());

    // runs reverse hopper (towards intake)
    operJoy.back()
        .onTrue(intake.setHopperSpeedCmd(-0.7))
        .onFalse(intake.stopHopperMotorCmd());

    /* SHOOTER ANGLE BUTTONS */
    // toggles arm manual
    operJoy.rightStick()
        .toggleOnTrue(shooterAngle.setManualAngleCmd(
            MathUtil.applyDeadband(-operJoy.getRightY(), 0.1)));

    /* * * SHOOTER WHEEL * * */
    // shooting -> positive
    operJoy.rightTrigger()
        .onTrue(shooter.shoot());

    // runs shooter intake -> negative
    operJoy.leftTrigger()
        .onTrue(
            new RunCommand(() -> shooterWheel.setSpeed(-0.5), shooterWheel))
        .onFalse(
            new InstantCommand(() -> shooterWheel.stopMotors(), shooterWheel));
    /* BEAM BREAK */
    // Trigger breaks = operJoy.back();
    // breaks
    // .onTrue(new InstantCommand(() -> beambreak.setEmitter(true), beambreak))
    // .onFalse(
    // new InstantCommand(() -> beambreak.setEmitter(false), beambreak)
    // );
    Trigger zeroGyro = driveJoy.rightBumper();
    zeroGyro
        .onTrue(new InstantCommand(
            () -> drivetrain.zeroHeading(), drivetrain));

    /* CLIMB BUTTONS */

    Trigger extendClimbButton = operJoy.povUp();
    extendClimbButton
        .onTrue(new RunCommand(() -> climb.extendClimbArm(), climb))
        .onFalse(new InstantCommand(() -> climb.stopClimb(), climb));

    Trigger retractClimbButton = operJoy.povDown();
    retractClimbButton
        .onTrue(new RunCommand(() -> climb.retractClimbArm(), climb))
        .onFalse(new InstantCommand(() -> climb.stopClimb(), climb));

    /* INTAKE BUTTONS */
    Trigger runIntake = operJoy.rightBumper(); // change buttons later
    runIntake
        .onTrue(new InstantCommand(
                () -> intake.setIntakeVelocitySetpoint(1.0 * 360.0),
                //() -> intake.setIntakeSpeed(IntakeConstants.ROLLER_SPEED),
                intake))
        .onFalse(new InstantCommand(
            () -> intake.setIntakeVelocitySetpoint(0.0), intake));

    // amp flush
    operJoy.a()
        .onTrue(
            shooterAngle.SetAngleSetpointCmd(ShooterAngleConstants.AMP_FLUSH)
            .alongWith(shooterWheel.setVelocitySetpointCmd(ShooterWheelConstants.AMP_FLUSH))
        );

    // speaker flush
    operJoy.x()
        .onTrue(
            shooterAngle.SetAngleSetpointCmd(ShooterAngleConstants.SPEAKER_FLUSH)
            .alongWith(shooterWheel.setVelocitySetpointCmd(ShooterWheelConstants.SPEAKER_FLUSH))
        );
    Trigger runOuttake = operJoy.leftBumper(); // change buttons later
    runOuttake
        .onTrue(
            new RunCommand(
                () -> intake.setIntakeSpeed(-IntakeConstants.ROLLER_SPEED),
                intake))
        .onFalse(new RunCommand(() -> intake.stopIntakeMotor(), intake));

    Trigger hopperQuasistaticButton = driveJoy.a();
    hopperQuasistaticButton.whileTrue(
    hopper.hopperQuas(SysIdRoutine.Direction.kForward));

    Trigger hopperDynamicButton = driveJoy.b();
    hopperDynamicButton.whileTrue(
    hopper.hopperDyna(SysIdRoutine.Direction.kForward));

    Trigger hopperQuasistaticRButton = driveJoy.x();
    hopperQuasistaticRButton.whileTrue(
    hopper.hopperQuas(SysIdRoutine.Direction.kReverse));

    Trigger hopperDynamicRButton = driveJoy.y();
    hopperDynamicRButton.whileTrue(
    hopper.hopperDyna(SysIdRoutine.Direction.kReverse));

    // speaker stage
    operJoy.y()
        .onTrue(
            shooterAngle.SetAngleSetpointCmd(ShooterAngleConstants.SPEAKER_STAGE)
            .alongWith(shooterWheel.setVelocitySetpointCmd(ShooterWheelConstants.SPEAKER_STAGE))    
        );

    // speaker wing
    operJoy.b()
        .onTrue(
            shooterAngle.SetAngleSetpointCmd(Constants.ShooterAngleConstants.AMP_FLUSH)
            .alongWith(shooterWheel.setVelocitySetpointCmd(ShooterWheelConstants.SPEAKER_WING))
        );
    // positive speed is outwards

    // Trigger fiveshootersetpoint = operJoy.leftTrigger();
    // fiveshootersetpoint
    // .onTrue(new InstantCommand(
    //     () -> shooterWheel.setVelocitySetpoint(5.0 * 360.0)
    // ));

    Trigger runShooterIntake = operJoy.leftTrigger();
    runShooterIntake
        .onTrue(
            new RunCommand(() -> shooterWheel.setShooterSpeed(-0.5), shooterWheel))
        .onFalse(
            new InstantCommand(() -> shooterWheel.stopShooter(), shooterWheel));
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    /* HOPPER BUTTONS */
    Trigger runHopper = operJoy.start(); // change buttons later
    runHopper
        .onTrue(new InstantCommand(
            () -> hopper.setHopperVelocitySetpoint(1.0 * 360), intake))
            //() -> intake.setHopperSpeed(1), intake)) // need to code for when it is
        .onFalse(new InstantCommand(
            () -> hopper.setHopperVelocitySetpoint(0), intake));
            //() -> intake.stopHopperMotor(), intake));

    Trigger runInverseHopper = operJoy.back();
    runInverseHopper
        .onTrue(new RunCommand(() -> hopper.setHopperSpeed(-0.7), intake))
        .onFalse(new InstantCommand(() -> hopper.stopHopperMotor(), intake));

    /* INTAKE ROUTINE */
    /* I AM NOT SURE IF THIS WILL WORK SOMEONE PLEASE CHECK IT*/
    Trigger runIntakeHopper = operJoy.x();
     runIntakeHopper
      .onTrue(
        new RunIntakeHopper(intake,hopper)
      );
      

    
    /* SHOOTER ANGLE BUTTONS */
    // Trigger twenty = operJoy.a();
    // twenty
    //     .onTrue(new InstantCommand(() -> shooterAngle.setAngleSetpoint(19.9), shooterAngle));

    // Trigger thirty = operJoy.b();
    // thirty
    //     .onTrue(new InstantCommand(() -> shooterAngle.setAngleSetpoint(25.0), shooterAngle));

    // Trigger fourtyFive = operJoy.y();
    // fourtyFive
    //     .onTrue(new InstantCommand(() -> shooterAngle.setAngleSetpoint(45.0), shooterAngle));

    // Trigger sixty = operJoy.x();
    // sixty
    //     .onTrue(new InstantCommand(() -> shooterAngle.setAngleSetpoint(60.0), shooterAngle));

    /* SHOOTER WHEEL BUTTONS */
        Trigger runShooter = operJoy.rightTrigger();
    runShooter
        .onTrue(
            new RunCommand(() -> 
            shooterWheel.setShooterSpeed(1), shooterWheel))
        .onFalse(
            new InstantCommand(() -> shooterWheel.stopShooter(), shooterWheel));
    Trigger zerorot = operJoy.a();
    zerorot
    .onTrue(new InstantCommand(
    () -> shooterWheel.setVelocitySetpoint(0), shooterWheel));

    Trigger tworot = operJoy.b();
    tworot
    .onTrue(new InstantCommand(
    () -> shooterWheel.setVelocitySetpoint(2.0 * 360.0), shooterWheel));

    Trigger fiverot = operJoy.x();
    fiverot
    .onTrue(new InstantCommand(
    () -> shooterWheel.setVelocitySetpoint(5.0 * 360.0), shooterWheel));

    Trigger tenrot = operJoy.y();
    tenrot
    .onTrue(new InstantCommand(
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

    /* INTAKE SYSID */
    // Trigger hopperQuasistaticButton = driveJoy.a();
    // hopperQuasistaticButton.whileTrue(
    // intake.intakeQuas(SysIdRoutine.Direction.kForward));

    // Trigger hopperDynamicButton = driveJoy.b();
    // hopperDynamicButton.whileTrue(
    // intake.intakeDyna(SysIdRoutine.Direction.kForward));

    // Trigger hopperQuasistaticRButton = driveJoy.x();
    // hopperQuasistaticRButton.whileTrue(
    // intake.intakeQuas(SysIdRoutine.Direction.kReverse));

    // Trigger hopperDynamicRButton = driveJoy.y();
    // hopperDynamicRButton.whileTrue(
    // intake.intakeDyna(SysIdRoutine.Direction.kReverse));

    /* INTAKE SYSID */
    // Trigger intakeQuasistaticButton = driveJoy.a();
    // intakeQuasistaticButton.whileTrue(
    // intake.intakeQuas(SysIdRoutine.Direction.kForward));

    // Trigger intakeDynamicButton = driveJoy.b();
    // intakeDynamicButton.whileTrue(
    // intake.intakeDyna(SysIdRoutine.Direction.kForward));

    // Trigger intakeQuasistaticRButton = driveJoy.x();
    // intakeQuasistaticRButton.whileTrue(
    // intake.intakeQuas(SysIdRoutine.Direction.kReverse));

    // Trigger intakeDynamicRButton = driveJoy.y();
    // intakeDynamicRButton.whileTrue(
    // intake.intakeDyna(SysIdRoutine.Direction.kReverse));
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
