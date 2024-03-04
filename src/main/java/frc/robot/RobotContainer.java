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
import frc.robot.Constants.ShooterAngleConstants;
import frc.robot.Constants.ShooterWheelConstants;
import frc.robot.DrivetrainConstants.OIConstants;
import frc.robot.Ports.*;
import frc.robot.commands.Controls;
import frc.robot.commands.Intaking;
import frc.robot.commands.Shooter;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hopper;
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

  private final Shooter shooter = new Shooter(shooterAngle, shooterWheel, hopper);
  private final Intaking intaking = new Intaking(intake, hopper);
  private final Controls controls = new Controls(shooterAngle, shooterWheel, hopper, intake, drivetrain);

  private final SendableChooser<Command> autonChooser = new SendableChooser<>();

  public RobotContainer() {
    // configurations
    configureButtonBindings();
    configureAuton();
    configureDefaultCommands();
  }

  public void configureDefaultCommands() {
    drivetrain.setDefaultCommand(
        // all joy.get values -> negative
        drivetrain.defaultCmd(
            -driveJoy.getRightY(), -driveJoy.getRightX(), -driveJoy.getLeftX(),
            true, true, OIConstants.DEADBAND)); // field rel = true


    shooterAngle.setDefaultCommand(
        // new RunCommand(
        //     () -> shooterAngle.setManualAngle(
        //         MathUtil.applyDeadband(-operJoy.getRightY(), 0.1)),
        //     shooterAngle));
        shooterAngle.setAngleCmd());

    shooterWheel.setDefaultCommand(shooterWheel.stopMotorsCmd());
  }
  
  public void configureAuton() {
    SmartDashboard.putData("Choose Auto: ", autonChooser);
    autonChooser.addOption("Shoot Amp", shooter.autonShoot(ShooterAngleConstants.AMP_FLUSH));
  }

  private void configureButtonBindings() {

    /* * * DRIVE BUTTONS * * */
        // reset gyro
        driveJoy.rightBumper()
            .onTrue(drivetrain.resetGyroCmd());

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
        // runs intake routine
        operJoy.rightBumper()
            // .onTrue(intake.setIntakeSpeedCmd(IntakeConstants.ROLLER_SPEED))
            // .onFalse(intake.stopMotorCmd());
            .onTrue(intake.setVelocitySetpointCmd(1.0 * 360.0)) // just the intake mech
            .onFalse(intake.setVelocitySetpointCmd(0.0));
            
            // entire intake routine
            // .onTrue(intaking.intakeNote());

        // runs outtake
        operJoy.leftBumper()
            .onTrue(intake.setVelocitySetpointCmd(-1.0 * 360.0))
            .onFalse(intake.setVelocitySetpointCmd(0.0));
            // .onTrue(intake.setSpeedCmd(-IntakeConstants.ROLLER_SPEED))
            // .onFalse(intake.stopMotorCmd());

    /* * * HOPPER BUTTONS * * */
        // runs hopper (towards shooter)
        operJoy.start()
            // .onTrue(hopper.setHopperSpeedCmd(0.7))
            // .onFalse(hopper.stopHopperMotorCmd());
            .onTrue(hopper.setVelocitySetpointCmd(360))
            .onFalse(hopper.setVelocitySetpointCmd(0));

        // runs reverse hopper (towards intake)
        operJoy.back()
            // .onTrue(hopper.setHopperSpeedCmd(-0.7))
            // .onFalse(hopper.stopHopperMotorCmd());
            .onTrue(hopper.setVelocitySetpointCmd(-360))
            .onFalse(hopper.setVelocitySetpointCmd(0));

    /* * * SHOOTER WHEEL * * */
        // shooting -> positive
        operJoy.rightTrigger()
            // .onTrue(shooter.shoot());
            // .onTrue(shooterWheel.setVelocitySetpointCmd(10.0*360).andThen(shooterWheel.setVelocityCmd().until(shooterWheel::atVelocity)));
        //     .onTrue(
        //         new ConditionalCommand(
        //     (shooterWheel.setVelocitySetpointCmd(10.0*360).andThen(shooterWheel.setVelocityCmd().until(shooterWheel::atVelocity))) // ramps shooter to desired velocity
        //         .andThen(hopper.setVelocitySetpointCmd(360).andThen(hopper.setHopperVelocityCmd())), // moves hopper after desired vel is reached
        //     (shooterWheel.stopMotorsCmd().alongWith(hopper.stopHopperMotorCmd())), // stops motors first
        //     hopper::isHopperEmpty // stops first command when hopper is ready
        // )
            // );
        .onTrue(hopper::isHopperEmpty);
        
        // runs shooter intake -> negative
        // TODO: CHANGE setSpeed to velocity later; FIGURE OUT SHOOTER INTAKE ROUTINE
        operJoy.leftTrigger()
            .onTrue(
                new RunCommand(() -> shooterWheel.setSpeed(-0.5), shooterWheel))
            .onFalse(
                new InstantCommand(() -> shooterWheel.stopMotors(), shooterWheel));

    /* * * SHOOTER ANGLE BUTTONS * * */
        // toggles arm manual
        operJoy.rightStick()
            .toggleOnTrue(shooterAngle.setManualAngleCmd(
                MathUtil.applyDeadband(-operJoy.getRightY(), 0.1)));

        // amp flush
        operJoy.a()
            .onTrue(shooter.setShooterSetpoints(ShooterAngleConstants.AMP_FLUSH, ShooterWheelConstants.AMP_FLUSH));

        // speaker flush
        operJoy.x()
            .onTrue(shooter.setShooterSetpoints(ShooterAngleConstants.SPEAKER_FLUSH, ShooterWheelConstants.SPEAKER_FLUSH));
            
        // speaker stage
        operJoy.y()
            .onTrue(shooter.setShooterSetpoints(ShooterAngleConstants.SPEAKER_STAGE, ShooterWheelConstants.SPEAKER_STAGE));

        // speaker wing
        operJoy.b()
            .onTrue(shooter.setShooterSetpoints(ShooterAngleConstants.SPEAKER_WING, ShooterWheelConstants.SPEAKER_WING));

    /* * * CONTROL BINDINGS * * */
        /*
         * controlTypes: pid, sysid
         * pid subsystems: shooterAngle, shooterWheel
         * sysid subsystems: hopper, intake, drivetrain
         * buttons: a, b, x, y, rightBumper, leftBumper
        */
        // driveJoy.a()
        //     .onTrue(controls.controlSwitch("pid", "shooterAngle", "a"));
        // driveJoy.b()
        //     .onTrue(controls.controlSwitch("pid", "shooterAngle", "b"));
        // driveJoy.x()
        //     .onTrue(controls.controlSwitch("pid", "shooterAngle", "x"));
        // driveJoy.y()
        //     .onTrue(controls.controlSwitch("pid", "shooterAngle", "y"));
        // driveJoy.rightBumper()
        //     .onTrue(controls.controlSwitch("sysid", "drivetrain", "rightBumper"));
        // driveJoy.leftBumper()
        //     .onTrue(controls.controlSwitch("sysid", "drivetrain", "leftBumper"));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autonChooser.getSelected();
  }
}
