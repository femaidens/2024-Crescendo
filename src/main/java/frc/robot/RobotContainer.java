// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeHopperConstants;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.ShooterAngleConstants;
import frc.robot.Constants.ShooterWheelConstants;
import frc.robot.DrivetrainConstants.DriveConstants;
import frc.robot.DrivetrainConstants.ModuleConstants;
import frc.robot.DrivetrainConstants.ModuleConstants.Drive;
import frc.robot.DrivetrainConstants.ModuleConstants.Turning;
import frc.robot.DrivetrainConstants.OIConstants;
import frc.robot.Ports.*;
// import frc.robot.autos.paths.Taxi;
// import frc.robot.autos.paths.TaxiAmp;
// import frc.robot.autos.paths.TaxiSpeaker;
// import frc.robot.commands.ThreeNoteFlushAuto;
// import frc.robot.autos.paths.TaxiSpeaker;
// import frc.robot.autos.AutoDrive;
import frc.robot.commands.Controls;
import frc.robot.commands.Intaking;
// import frc.robot.commands.OneNoteLeft;
import frc.robot.commands.Shooter;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ShooterAngle;
import frc.robot.subsystems.ShooterWheel;
import monologue.Logged;

import org.littletonrobotics.urcl.URCL;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

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
  private final LED led = new LED();

  private final Shooter shooter = new Shooter(shooterAngle, shooterWheel, hopper, led);
  private final Intaking intaking = new Intaking(intake, hopper, led);
  private final Controls controls = new Controls(shooterAngle, shooterWheel, hopper, intake, drivetrain);


  private SendableChooser<Command> autonChooser;

//   autonChooser.addOption("Three Note Flush Auto", new ThreeNoteFlushAuto());

  public RobotContainer() {
    // configurations
    configureButtonBindings();
    configureAuton();

    autonChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Choose Auto: ", autonChooser);
    
    configureDefaultCommands();
  }

  public void configureSubsystemDefaults() {
    // drivetrain.resetGyroCmd();
    // drivetrain.resetEncoders();
    // shooterAngle.setAngleSetpointCmd(ShooterAngleConstants.INITIAL_ANGLE); -> did this in subsystem
  }

  public void configureDefaultCommands() {

//    drivetrain.setDefaultCommand(
//     //  clarify turning with right or with left
//       new RunCommand(
//           () -> drivetrain.drive( // all joy.get values were prev negative
//               MathUtil.applyDeadband(-driveJoy.getLeftY(), 0.1),
//               MathUtil.applyDeadband(-driveJoy.getLeftX(), 0.1),
//               MathUtil.applyDeadband(-driveJoy.getRightX(), 0.1),
//               true, false),
//           drivetrain)
//     );

    shooterAngle.setDefaultCommand(
        new RunCommand(
            () -> shooterAngle.setManualAngle(
                MathUtil.applyDeadband(-operJoy.getRightY(), 0.1)),
            shooterAngle)
    );

    // shooterWheel.setDefaultCommand(shooterWheel.setVelocityCmd(ShooterWheelConstants.DEFAULT_VELOCITY));
    // shooterWheel.setDefaultCommand(shooterWheel.setVelocityCmd(ShooterWheelConstants.DEFAULT_VELOCITY));
    shooterWheel.setDefaultCommand(shooterWheel.setVelocityCmd());

    // // if default velocity is 0, need to run command when scheduling the command
    // // if not, make sure that setpoints are changing correctly
    hopper.setDefaultCommand(hopper.setVelocityCmd());
    intake.setDefaultCommand(intake.setVelocityCmd());
    // leds.setDefaultCommand(leds.setRainbowCmd());
    led.setDefaultCommand(
        led.setPurpGreenCmd().andThen(new WaitCommand(3))
    );
  }
  
  public void configureAuton() {
    // autonChooser.addOption("taxi", new Taxi(drivetrain, hopper, shooterAngle, shooterWheel, AutoConstants.DRIVE_TIME));
    // autonChooser.addOption("taxi amp", new TaxiAmp(drivetrain, hopper, shooterAngle, shooterWheel));
    // autonChooser.addOption("taxi speaker", new TaxiSpeaker(drivetrain, hopper, shooterAngle, shooterWheel));
    
    // autonChooser = AutonBuilder.
    // autonChooser.addOption("flush three notes", new ThreeNoteFlushAuto(drivetrain, intaking, shooter));
    // autonChooser.addOption("flush one note", new OneNoteLeft(drivetrain, intaking, shooter));
    NamedCommands.registerCommand("ampshoot", shooter.shoot(ShooterAngleConstants.AMP_FLUSH, ShooterWheelConstants.AMP_FLUSH));
    NamedCommands.registerCommand("speakerstageshoot", shooter.shoot(ShooterAngleConstants.SPEAKER_STAGE, ShooterWheelConstants.SPEAKER_STAGE));
    NamedCommands.registerCommand("intake", intaking.intakeNote());
    NamedCommands.registerCommand("speakerflushshoot", shooter.shoot(ShooterAngleConstants.SPEAKER_FLUSH, ShooterWheelConstants.SPEAKER_FLUSH).withTimeout(3.0));
    // AutoBuilder autoBuilder = new AutoBuilder();
        AutoBuilder.configureHolonomic(
        drivetrain::getPose, 
        drivetrain::resetOdometry, 
        drivetrain::getRobotRelativeChassisSpeeds, //chassis speed supplier must be robot relative
        drivetrain::setChassisSpeeds, //method that will drive the robot based on robot relative chassis speed
        new HolonomicPathFollowerConfig(
            new PIDConstants(Drive.kP, Drive.kI, Drive.kD), // Translation PID constants
            new PIDConstants(Turning.kP, Turning.kI, Turning.kD), // Rotation PID constants
            DriveConstants.MAX_SPEED, // Max module speed, in m/s
            ModuleConstants.WHEEL_DIAMETER/2, 
            new ReplanningConfig()), 
        () -> {
        var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
        drivetrain);

  }

  private void configureButtonBindings() {

    /* * * DRIVE BUTTONS * * */
    //     // reset gyro
    //     driveJoy.rightBumper()
    //         .onTrue(drivetrain.resetGyroCmd());

    //     // speed factor
    //     driveJoy.leftTrigger()
    //         .onTrue(drivetrain.slowCmd())
    //         .onFalse(drivetrain.regularCmd());
        
        // tests led after trigger is triggered -> works!
        driveJoy.start()
            .onTrue(
                // intaking.setIntakeHopperSetpoints(0)
                led.setRedCmd().until(hopper::isHopperFull).andThen(led.setGreenCmd().withTimeout(3)) // -> works
                // Commands.waitUntil(hopper::isHopperFull)
                // .andThen(leds.setGreenCmd().withTimeout(3))
                // cannot put the withTimeout outside otherwise, it gives it 3 secs for the entier thing)
            );

    /* SHOOTER ANGLE SETPOINTS */
        // driveJoy.a()
        //   .onTrue(shooterAngle.setAngleSetpointCmd(28));

        // driveJoy.b()
        //   .onTrue(shooterAngle.setAngleSetpointCmd(35));

        // driveJoy.x()
        //   .onTrue(shooterAngle.setAngleSetpointCmd(50));

        // driveJoy.y()
        //   .onTrue(shooterAngle.setAngleSetpointCmd(60));

    /* * * CLIMB BUTTONS * * */
        // extend climb arm
        operJoy.povUp()
            .onTrue(climb.extendClimbCmd())
            .onFalse(climb.stopMotorsCmd());

    //     // retract climb arm
    //     operJoy.povDown()
    //         .onTrue(climb.retractClimbCmd())
    //         .onFalse(climb.stopMotorsCmd());

    /* * * INTAKE BUTTONS * * */
        // runs intake routine
        operJoy.rightBumper()
            // test entire routine
            .onTrue(shooterAngle.setAngleSetpointCmd(ShooterAngleConstants.MIN_ANGLE)
            .andThen(intaking.intakeNote()));

            // entire intake routine with setSpeed
            // .onTrue(leds.setSolidCmd(LEDConstants.RED) // test led red
            //     .andThen(intaking.setIntakeHopperSpeeds(0.3)) // bc it's a runOnce, it automatically went to setting sp to 0
            //     .andThen(Commands.waitUntil(hopper::isHopperFull))
            //     .andThen(intake.stopMotorCmd().alongWith(hopper.stopMotorCmd())) 
            //     // .andThen(() -> hopper.resetStateCountCmd()) // testing, commented out before
            //     .andThen(leds.setGreenCmd().withTimeout(2))
            //     // .finallyDo(() -> leds.setLedGreen()).withTimeout(2) // doesn't work
            // );
            
            // entire intake routine with setVelocity
            // .onTrue(intaking.moveNote(IntakeHopperConstants.INTAKE_NOTE_SPEED) // bc it's a runOnce, it automatically went to setting sp to 0
            //     .andThen(Commands.waitUntil(hopper::isHopperFull))
            //     .andThen(intaking.setIntakeHopperSetpoints(0))
            //     // .andThen(() -> hopper.resetStateCountCmd()) // testing, commented out before
            //     .andThen(leds.setGreenCmd().withTimeout(2))
            //     // .finallyDo(() -> leds.setLedGreen()).withTimeout(2) // doesn't work
            // );

    //         // unit testing 
    //         // intaking intakeHopper work
    //         // .onTrue(intaking.setIntakeHopperSetpoints(3*360))
    //         // .onFalse(intaking.setIntakeHopperSetpoints(0));
            
        // runs outtake
        operJoy.leftBumper()
            // setVelocity entire outtake routine
            // .onTrue(intaking.setIntakeHopperSetpoints(-IntakeHopperConstants.INTAKE_NOTE_SPEED))
            // .onFalse(intaking.setIntakeHopperSetpoints(0));

            // setSpeed entire outtake routines
            .onTrue(shooterAngle.setAngleSetpointCmd(ShooterAngleConstants.MIN_ANGLE)
                .andThen(intaking.setOuttakeSpeeds(-0.3)))
            .onFalse(intaking.setOuttakeSpeeds(0));

    /* * * HOPPER BUTTONS * * */
        // runs hopper (towards shooter) --> failsafe if the trigger process fails
        operJoy.start()
            .onTrue(hopper.setVelocitySetpointCmd(HopperConstants.TRANSITION_VEL))
            .onFalse(hopper.setVelocitySetpointCmd(0));

        // feeds note from hopper to shooter aka "trigger"
        // operJoy.back()
            // test the feedNote command
            // .onTrue(hopper.feedNote());

            // entire trigger process
            // .onTrue(
            //     hopper.feedNote()
            //     .andThen(shooterWheel.setVelocitySetpoint(0))
            //     .andThen(hopper.resetStateCountCmd())
            //     .andThen(led.setSolid(LEDConstants.GREEN).withTimeout(2))
            // )

            // og entire trigger process; if feedNote doesn't work
            // .onTrue(Commands.waitUntil(() -> hopper.isHopperFull())
            //     .andThen(hopper.setVelocitySetpointCmd(HopperConstants.TRANSITION_SPEED))
            //     .andThen(Commands.waitUntil(hopper::isHopperEmpty))
            //     .andThen(hopper.setVelocitySetpointCmd(0)) // where feedNote ends
            //     .andThen(shooterWheel.setVelocitySetpointCmd(0))
            //     .andThen(hopper.resetStateCountCmd()) // testing, commented in before
            //     .andThen(led.setSolidCmd(LEDConstants.GREEN).withTimeout(2))
            // );

    /* * * SHOOTER WHEEL * * */
        // shooting -> positive
        // voltage ramping shooter
        operJoy.rightTrigger()
            // .onTrue(hopper.resetStateEmergencyCmd()
            .onTrue(shooter.shoot()

            // failsafe if shooting with abxy doesn't work
            // .onTrue(shooter.shoot());
        );
            // just run shooter
            // .onTrue(shooterWheel.setVelocityCmd(ShooterWheelConstants.AMP_FLUSH));
        
        // runs shooter intake -> negative
        operJoy.leftTrigger()
            // reset to default config
            .onTrue(
                shooterAngle.setAngleSetpointCmd(ShooterAngleConstants.SPEAKER_FLUSH) // amp angle
                // .alongWith(intaking.setIntakeHopperSetpoints(0))
                .alongWith(shooterWheel.setVelocitySetpointCmd(0))
                .alongWith(hopper.resetStateCountCmd()) // check to see if this works
            );

    // /* * * SHOOTER ANGLE BUTTONS * * */
    //     // toggles arm manual -> made default command
    //     // operJoy.rightStick()
    //     //     .toggleOnTrue(shooterAngle.setManualAngleCmd(
    //     //         MathUtil.applyDeadband(-operJoy.getRightY(), 0.1)));

    //     // amp flush
    //     operJoy.a()
    //         .onTrue(shooter.setShooterSetpoints(ShooterAngleConstants.AMP_FLUSH, ShooterWheelConstants.AMP_FLUSH)
    //         .alongWith(hopper.setStateLimitCmd(2))
    //         );

            // shooting with buttons
            // .onTrue(hopper.setStateLimitCmd(2)
            //     .alongWith(shooter.shoot(ShooterAngleConstants.AMP_FLUSH, ShooterWheelConstants.AMP_FLUSH))
            // );

    //         // testing at angle 
    //         // .onTrue(shooterAngle.setAngleSetpointCmd(ShooterAngleConstants.AMP_FLUSH));
        
        // speaker flush
        operJoy.x()
            // .onTrue(shooter.setShooterSetpoints(ShooterAngleConstants.SPEAKER_FLUSH, ShooterWheelConstants.SPEAKER_FLUSH));
            // .alongWith(hopper.setStateLimitCmd(1))
            // );
            
            // shooting with buttons
            .onTrue(hopper.setStateLimitCmd(1)
                .alongWith(shooter.shoot(ShooterAngleConstants.SPEAKER_FLUSH, ShooterWheelConstants.SPEAKER_FLUSH))
            );
            
    //         // testing at angle 
    //         // .onTrue(shooterAngle.setAngleSetpointCmd(ShooterAngleConstants.SPEAKER_FLUSH));

        // speaker stage
        operJoy.y()
            // .onTrue(shooter.setShooterSetpoints(ShooterAngleConstants.SPEAKER_STAGE, ShooterWheelConstants.SPEAKER_STAGE)
            // .alongWith(hopper.setStateLimitCmd(1))
            // );
            
            // shooting with buttons
            // .onTrue(hopper.setStateLimitCmd(1)
            //     .alongWith(shooter.shoot(ShooterAngleConstants.SPEAKER_STAGE, ShooterWheelConstants.SPEAKER_STAGE))
            // );

            // just setting angle and state limit
            .onTrue(hopper.setStateLimitCmd(1)
                .alongWith(shooterAngle.setAngleSetpointCmd(ShooterAngleConstants.SPEAKER_STAGE))
            );
            
    //         // testing at angle
    //         // .onTrue(shooterAngle.setAngleSetpointCmd(ShooterAngleConstants.SPEAKER_STAGE));

    //     // // speaker wing
    //     // operJoy.b()
    //     //     .onTrue(shooter.setShooterSetpoints(ShooterAngleConstants.SPEAKER_STAGE +3 , ShooterWheelConstants.SPEAKER_STAGE));
    //     //     .onTrue(shooterAngle.setAngleSetpointCmd(ShooterAngleConstants.SPEAKER_WING)
    //     //         .alongWith(shooterWheel.setVelocitySetpointCmd(ShooterWheelConstants.SPEAKER_WING)));

    /* * * CONTROL BINDINGS * * */
    // driveJoy.a()
    // .whileTrue(
    //     shooterAngle.quasiCmd(SysIdRoutine.Direction.kForward).until(shooterAngle::atMaxAngle)
    // );

    // driveJoy.b()
    // .whileTrue(
    //     shooterAngle.quasiCmd(SysIdRoutine.Direction.kReverse).until(shooterAngle::atMinAngle)
    // );
    // driveJoy.x()
    // .whileTrue(
    //     shooterAngle.dynaCmd(SysIdRoutine.Direction.kForward).until(shooterAngle::atMaxAngle)
    // );
    // driveJoy.y()
    // .whileTrue(
    //     shooterAngle.dynaCmd(SysIdRoutine.Direction.kReverse).until(shooterAngle::atMinAngle)
    // );

    /* DRIVETRAIN SYSID */
    // driveJoy.a()
    //     .whileTrue(
    //         drivetrain.driveQuasistatic(Direction.kForward)
    //     );

    // driveJoy.b()
    //     .whileTrue(
    //         drivetrain.driveQuasistatic(Direction.kReverse)
    //     );

    // driveJoy.x()
    //     .whileTrue(
    //         drivetrain.driveDynamic(Direction.kForward)
    //     );

    // driveJoy.y()
    //     .whileTrue(
    //         drivetrain.driveDynamic(Direction.kReverse)
    //     );
        
    /* DRIVETRAIN TURNING SYSID */
    // driveJoy.a()
    //     .whileTrue(
    //         drivetrain.turnQuasistatic(Direction.kForward)
    //     );

    // driveJoy.b()
    //     .whileTrue(
    //         drivetrain.turnQuasistatic(Direction.kReverse)
    //     );

    // driveJoy.x()
    //     .whileTrue(
    //         drivetrain.turnDynamic(Direction.kForward)
    //     );

    // driveJoy.y()
    //     .whileTrue(
    //         drivetrain.turnDynamic(Direction.kReverse)
    //     );

    /* SHOOTER ANGLE SYSID */
    // driveJoy.a()
    // .whileTrue(
    //     shooterAngle.quasiCmd(SysIdRoutine.Direction.kForward).until(shooterAngle::atMaxAngle)
    // );

    // driveJoy.b()
    // .whileTrue(
    //     shooterAngle.quasiCmd(SysIdRoutine.Direction.kReverse).until(shooterAngle::atMinAngle)
    // );
    // driveJoy.x()
    // .whileTrue(
    //     shooterAngle.dynaCmd(SysIdRoutine.Direction.kForward).until(shooterAngle::atMaxAngle)
    // );
    // driveJoy.y()
    // .whileTrue(
    //     shooterAngle.dynaCmd(SysIdRoutine.Direction.kReverse).until(shooterAngle::atMinAngle)
    // );

    //     /*
    //      * controlTypes: pid, sysid
    //      * pid subsystems: shooterAngle, shooterWheel
    //      * sysid subsystems: hopper, intake, drivetrain
    //      * buttons: a, b, x, y, rightBumper, leftBumper
    //     */
    //     // driveJoy.a()
    //     //     .onTrue(controls.controlSwitch("pid", "shooterAngle", "a"));
    //     // driveJoy.b()
    //     //     .onTrue(controls.controlSwitch("pid", "shooterAngle", "b"));
    //     // driveJoy.x()
    //     //     .onTrue(controls.controlSwitch("pid", "shooterAngle", "x"));
    //     // driveJoy.y()
    //     //     .onTrue(controls.controlSwitch("pid", "shooterAngle", "y"));
    //     // driveJoy.rightBumper()
    //     //     .onTrue(controls.controlSwitch("sysid", "drivetrain", "rightBumper"));
    //     // driveJoy.leftBumper()
    //     //     .onTrue(controls.controlSwitch("sysid", "drivetrain", "leftBumper"));
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