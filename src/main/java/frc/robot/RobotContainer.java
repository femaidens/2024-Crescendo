// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeHopperConstants;
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
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ShooterAngle;
import frc.robot.subsystems.ShooterWheel;
import monologue.Logged;

import org.littletonrobotics.urcl.URCL;

public class RobotContainer implements Logged {

  private CommandXboxController driveJoy = new CommandXboxController(JoystickPorts.DRIVE_JOY);
  private CommandXboxController operJoy = new CommandXboxController(JoystickPorts.OPER_JOY);

  private final Drivetrain drivetrain = new Drivetrain();
  private final Intake intake = new Intake();
  private final Hopper hopper = new Hopper();
  private final ShooterWheel shooterWheel = new ShooterWheel();
  private final ShooterAngle shooterAngle = new ShooterAngle();
  private final Climb climb = new Climb();
  private final Limelight limelight = new Limelight();

//   private final Shooter shooter = new Shooter(shooterAngle, shooterWheel, hopper);
  private final Intaking intaking = new Intaking(intake, hopper);
//   private final Controls controls = new Controls(shooterAngle, shooterWheel, hopper, intake, drivetrain);

  private final SendableChooser<Command> autonChooser = new SendableChooser<>();

  public RobotContainer() {
    // configurations
    configureButtonBindings();
    configureAuton();
    configureDefaultCommands();
  }

  public void configureDefaultCommands() {
    // drivetrain.setDefaultCommand(
    //     // all joy.get values -> negative
    //     drivetrain.defaultCmd(
    //         -driveJoy.getRightY(), -driveJoy.getRightX(), -driveJoy.getLeftX(),
    //         true, true, OIConstants.DEADBAND)); // field rel = true

    drivetrain.setDefaultCommand(
     // clariy turning with right or with left
      new RunCommand(
          () -> drivetrain.drive( // all joy.get values were prev negative
              MathUtil.applyDeadband(-driveJoy.getLeftY(), 0.1),
              MathUtil.applyDeadband(-driveJoy.getLeftX(), 0.1),
              MathUtil.applyDeadband(-driveJoy.getRightX(), 0.1),
              true, true),
          drivetrain)); // field rel = true

    shooterAngle.setDefaultCommand(
        new RunCommand(
            () -> shooterAngle.setManualAngle(
                MathUtil.applyDeadband(-operJoy.getRightY(), 0.1)),
            shooterAngle));
        // shooterAngle.setAngleCmd());

    // shooterWheel.setDefaultCommand(shooterWheel.setVelocityCmd(ShooterWheelConstants.DEFAULT_VELOCITY));
    shooterWheel.setDefaultCommand(shooterWheel.setVelocityCmd());

    hopper.setDefaultCommand(hopper.setVelocityCmd());
    intake.setDefaultCommand(intake.setVelocityCmd());
  }
  
  public void configureAuton() {
    SmartDashboard.putData("Choose Auto: ", autonChooser);
    // autonChooser.addOption("Shoot Amp", shooter.autonShoot(AutoConstants.WHEEL_SPEED, ShooterAngleConstants.AMP_FLUSH));
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
            // comment out default command when running just speed
            // .onTrue(intake.setSpeedCmd(IntakeConstants.ROLLER_SPEED))
            // .onFalse(intake.stopMotorCmd());

            // just intake mech
            // .onTrue(intake.setVelocitySetpointCmd(IntakeConstants.INTAKE_VEL))
            // .onFalse(intake.setVelocitySetpointCmd(0.0));
            
            // entire intake routine
            // .onTrue(intaking.moveNote(IntakeHopperConstants.INTAKE_NOTE_SPEED));
            // .onTrue(intake.setVelocitySetpointCmd(IntakeHopperConstants.INTAKE_NOTE_SPEED)
            //     .alongWith(hopper.setVelocitySetpointCmd(IntakeHopperConstants.INTAKE_NOTE_SPEED))
            //     .waitUntil(()-> hopper.isHopperFull())
            //     .andThen(intake.setVelocitySetpoint(0))
            //     .alongWith(hopper.setVelocitySetpoint(0))
            // );
            
            // to test, just stick hand in; the motors should stop running
            // .onTrue(Commands.waitUntil(hopper::isHopperEmpty)
            //     .andThen(Commands.waitUntil(hopper::isHopperFull))
            //     .deadlineWith(intaking.setIntakeHopperSetpoints(IntakeHopperConstants.INTAKE_NOTE_SPEED))
            //     .finallyDo(() -> intaking.setIntakeHopperSetpoints(0))
            // );

            .onTrue(
                intaking.setIntakeHopperSetpoints(IntakeHopperConstants.INTAKE_NOTE_SPEED)
                .until(hopper::isHopperFull)
                .andThen(intaking.setIntakeHopperSetpoints(0))
            );
            
            // deadline and ishopperfull is the cut conditions
        // runs outtake
        operJoy.leftBumper()
            // .onTrue(intake.setVelocitySetpointCmd(-IntakeConstants.OUTTAKE_VEL))
            // .onFalse(intake.setVelocitySetpointCmd(0.0));
            
            // entire outtake routine
            // .onTrue(intaking.moveNote(-IntakeHopperConstants.INTAKE_NOTE_SPEED));

            .onTrue(intake.setSpeedCmd(-IntakeConstants.ROLLER_SPEED))
            .onFalse(intake.stopMotorCmd());

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
            // .onTrue(hopper.setVelocitySetpointCmd(-360))
            // .onFalse(hopper.setVelocitySetpointCmd(0));
            // .onTrue(hopper.feedNote());

            // feeds shooter; to test, stick note in first
            // .onTrue(Commands.waitUntil(() -> hopper.isHopperFull())
            //     .beforeStarting(hopper.resetStateCountCmd())
            //     .andThen(Commands.waitUntil(() -> hopper.isHopperEmpty())) // denotes when cmd ends
            //     .deadlineWith(hopper.setVelocitySetpointCmd(1*360.0))// runs hopper motors until note has been fed into shooter
            //     .finallyDo(() -> hopper.setVelocitySetpointCmd(0))
            // );

            .onTrue(Commands.waitUntil(() -> hopper.isHopperFull())
                .beforeStarting(hopper.resetStateCountCmd())
                .andThen(hopper.setVelocitySetpointCmd(360.0))
                .until(() -> hopper.isHopperEmpty())
                .finallyDo(() -> hopper.setVelocitySetpointCmd(0))
            );


    /* * * SHOOTER WHEEL * * */
        // shooting -> positive
        operJoy.rightTrigger()
            // .onTrue(shooter.shoot(shooterWheel.getSetpoint()));
            // .onTrue(shooter.shoot(25.0*360));
            .onTrue(shooterWheel.setVelocityCmd(12.0*360));
            // .onTrue(hopper.feedNote());
        
        // runs shooter intake -> negative
        // TODO: CHANGE setSpeed to velocity later; FIGURE OUT SHOOTER INTAKE ROUTINE
        operJoy.leftTrigger()
            .onTrue(
                new RunCommand(() -> shooterWheel.setSpeed(-0.5), shooterWheel))
            .onFalse(
                new InstantCommand(() -> shooterWheel.stopMotors(), shooterWheel));

    /* * * SHOOTER ANGLE BUTTONS * * */
        // toggles arm manual -> made default command
        // operJoy.rightStick()
        //     .toggleOnTrue(shooterAngle.setManualAngleCmd(
        //         MathUtil.applyDeadband(-operJoy.getRightY(), 0.1)));

        // amp flush
        operJoy.a()
            // .onTrue(shooter.setShooterSetpoints(ShooterAngleConstants.AMP_FLUSH, ShooterWheelConstants.AMP_FLUSH));
            .onTrue(shooterAngle.setAngleSetpointCmd(ShooterAngleConstants.AMP_FLUSH)
                .alongWith(shooterWheel.setVelocitySetpointCmd(ShooterWheelConstants.AMP_FLUSH)));

        // speaker flush
        operJoy.x()
            // .onTrue(shooter.setShooterSetpoints(ShooterAngleConstants.SPEAKER_FLUSH, ShooterWheelConstants.SPEAKER_FLUSH));
            .onTrue(shooterAngle.setAngleSetpointCmd(ShooterAngleConstants.SPEAKER_FLUSH)
                .alongWith(shooterWheel.setVelocitySetpointCmd(ShooterWheelConstants.SPEAKER_FLUSH)));
            
        // speaker stage
        operJoy.y()
            // .onTrue(shooter.setShooterSetpoints(ShooterAngleConstants.SPEAKER_STAGE, ShooterWheelConstants.SPEAKER_STAGE));
            .onTrue(shooterAngle.setAngleSetpointCmd(ShooterAngleConstants.SPEAKER_STAGE)
                .alongWith(shooterWheel.setVelocitySetpointCmd(ShooterWheelConstants.SPEAKER_STAGE)));

        // speaker wing
        operJoy.b()
            // .onTrue(shooter.setShooterSetpoints(ShooterAngleConstants.SPEAKER_WING, ShooterWheelConstants.SPEAKER_WING));
            .onTrue(shooterAngle.setAngleSetpointCmd(ShooterAngleConstants.SPEAKER_WING)
                .alongWith(shooterWheel.setVelocitySetpointCmd(ShooterWheelConstants.SPEAKER_WING)));

    /* * * CONTROL BINDINGS * * */
        // driveJoy.a().onTrue(shooterAngle.setAngleSetpointCmd(53));
        // driveJoy.b().onTrue(shooterAngle.setAngleSetpointCmd(63));
        // driveJoy.x().onTrue(shooterAngle.setAngleSetpointCmd(33));

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
