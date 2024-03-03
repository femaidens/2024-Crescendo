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
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ShooterAngle;
import frc.robot.subsystems.ShooterWheel;

import org.littletonrobotics.urcl.URCL;

public class RobotContainer {

  private CommandXboxController driveJoy = new CommandXboxController(JoystickPorts.DRIVE_JOY);
  private CommandXboxController operJoy = new CommandXboxController(JoystickPorts.OPER_JOY);

  private final Drivetrain drivetrain = new Drivetrain();
  private final Intake intake = new Intake();
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
        new RunCommand(() -> shooterWheel.stopMotors(), shooterWheel));
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
