// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterAngleConstants;
import frc.robot.Constants.ShooterWheelConstants;
import frc.robot.Constants.LEDConstants;
import frc.robot.Ports.*;
import org.littletonrobotics.urcl.URCL;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ShooterWheel;
import frc.robot.subsystems.ShooterAngle;
import frc.robot.subsystems.LED;

import frc.robot.commands.Controls;
import frc.robot.commands.Intaking;
import frc.robot.commands.Shooter;
import frc.robot.subsystems.Climb;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.DrivetrainConstants.OIConstants;


public class RobotContainer {

  private CommandXboxController driveJoy = new CommandXboxController(JoystickPorts.DRIVE_JOY);
  private CommandXboxController operJoy = new CommandXboxController(JoystickPorts.OPER_JOY);

  private final Drivetrain drivetrain = new Drivetrain();
  private final Intake intake = new Intake();
  private final Hopper hopper = new Hopper();
  private final ShooterWheel shooterWheel = new ShooterWheel();
  private final ShooterAngle shooterAngle = new ShooterAngle();
  private final Climb climb = new Climb();
  private final LED led = new LED(); 

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

        // default drive(gruple flicker) IDK if this will work :/ 
     led.setDefaultCommand(new RunCommand(() -> led.setGrupleFlicker(true), led));
      
  }
  
  public void configureAuton() {
    SmartDashboard.putData("Choose Auto: ", autonChooser);
    autonChooser.addOption("Shoot Amp", shooter.autonShoot(ShooterAngleConstants.AMP_FLUSH));
  }

  private void configureButtonBindings() {
    /* * * LEDS * * */
        // gruple = default drive thing  
        // driveJoy.a() 
        //     .onTrue(new RunCommand(
        //         () -> led.setGrupleFlicker(true), led)); 
       //READ THIS : this "Gruple Chase" does not have the right colors in it currently.
       // We could not figure out how to make it both green and purple, so currently it is merely green chasing green 
       // If you have time, pls try to change the parameters of it in LED to try and get purple + green
       // If unable, just comment the one below out, and use the one above for our default driving TY! <3 
        driveJoy.a() 
            .onTrue(new RunCommand(
                () -> led.setGrupleChase(), led)); 

        // solid purple = shoot, blue = climb, pink = intake 
        driveJoy.b()
            .onTrue( new RunCommand(
                () -> led.setSolid(LEDConstants.PURPLE), led));

        // rainbow = game over 
        driveJoy.x()
            .onTrue(new RunCommand(
                () -> led.setRainbow(), led));

        // red flicker = any error :()
        driveJoy.y()
            .onTrue(new RunCommand(
                () -> led.setRed(), led)); 

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
            .onTrue(shooter.shoot());
        
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
