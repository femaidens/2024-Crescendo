// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeHopperConstants;
import frc.robot.Constants.ShooterAngleConstants;
import frc.robot.Constants.ShooterWheelConstants;
import frc.robot.DrivetrainConstants.DriveConstants;
import frc.robot.DrivetrainConstants.ModuleConstants;
import frc.robot.DrivetrainConstants.OIConstants;
import frc.robot.DrivetrainConstants.ModuleConstants.Drive;
import frc.robot.DrivetrainConstants.ModuleConstants.Turning;
import frc.robot.Ports.*;
import frc.robot.autos.paths.Taxi;
import frc.robot.autos.paths.TaxiAmp;
import frc.robot.autos.paths.TaxiSpeaker;
// import frc.robot.autos.paths.TaxiSpeaker;
// import frc.robot.autos.AutoDrive;
import frc.robot.commands.Controls;
import frc.robot.commands.Intaking;
import frc.robot.commands.Shooter;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ShooterAngle;
import frc.robot.subsystems.ShooterWheel;
import monologue.Annotations.Log;
import monologue.Logged;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

import java.util.List;

import org.littletonrobotics.urcl.URCL;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
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
  private final LED leds = new LED();

  private final Shooter shooter = new Shooter(shooterAngle, shooterWheel, hopper);
  private final Intaking intaking = new Intaking(intake, hopper);
  private final Controls controls = new Controls(shooterAngle, shooterWheel, hopper, intake, drivetrain);

//   private final SendableChooser<Command> autonChooser = new SendableChooser<>();
//   private final SendableChooser<Command> allianceChooser = new SendableChooser<>();

//   private SendableChooser<Command> pathplannerChooser = new SendableChooser<>();

// @Log.NT private final SendableChooser<Command> autos;
  @Log.NT private SendableChooser<Command> pathplanner = new SendableChooser<>();
  public RobotContainer() {
    // configurations
    configureButtonBindings();
    configureAuton();
    configureDefaultCommands();
    // autos = AutoBuilder.buildAutoChooser();
    // SmartDashboard.putData("Auto Chooser", autos);

    pathplanner = AutoBuilder.buildAutoChooser();
    // SmartDashboard.putData("Pathplanner Auto Mode", pathplannerChooser);

  }

  public void configureSubsystemDefaults() {
    // drivetrain.resetGyroCmd();
    // drivetrain.resetEncoders();
    // shooterAngle.setAngleSetpointCmd(ShooterAngleConstants.INITIAL_ANGLE); -> did this in subsystem
  }

  public void configureDefaultCommands() {

    drivetrain.setDefaultCommand(
     // clariy turning with right or with left
      new RunCommand(
          () -> drivetrain.drive( // all joy.get values were prev negative
              MathUtil.applyDeadband(-driveJoy.getLeftY(), 0.1),
              MathUtil.applyDeadband(-driveJoy.getLeftX(), 0.1),
              MathUtil.applyDeadband(-driveJoy.getRightX(), 0.1),
              true, false),
          drivetrain)
    );

    shooterAngle.setDefaultCommand(
        new RunCommand(
            () -> shooterAngle.setManualAngle(
                MathUtil.applyDeadband(-operJoy.getRightY(), 0.1)),
            shooterAngle)
    );

    shooterWheel.setDefaultCommand(shooterWheel.setVelocityCmd(ShooterWheelConstants.DEFAULT_VELOCITY));
    shooterWheel.setDefaultCommand(shooterWheel.setVelocityCmd());

    // // if default velocity is 0, need to run command when scheduling the command
    // // if not, make sure that setpoints are changing correctly
    hopper.setDefaultCommand(hopper.setVelocityCmd());
    intake.setDefaultCommand(intake.setVelocityCmd());
    leds.setDefaultCommand(leds.setRainbowCmd());
  }
  
  public void configureAuton() {
    // SmartDashboard.putData("Choose Auto: ", autonChooser);
    // autonChooser.addOption("taxi", new Taxi(drivetrain, hopper, shooterAngle, shooterWheel, AutoConstants.DRIVE_TIME));
    // autonChooser.addOption("taxi amp", new TaxiAmp(drivetrain, hopper, shooterAngle, shooterWheel));
    // autonChooser.addOption("taxi speaker", new TaxiSpeaker(drivetrain, hopper, shooterAngle, shooterWheel));

    SmartDashboard.putData("Choose Pathplanner Auto: ", pathplanner);
    NamedCommands.registerCommand("shoot", shooter.shoot());
    NamedCommands.registerCommand("intake", intaking.moveNote(IntakeHopperConstants.INTAKE_NOTE_SPEED));

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
            new ReplanningConfig()
        ), 
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
    // RobotModeTriggers.autonomous().whileTrue(Commands.deferredProxy(pathplanner::getSelected));

    RobotModeTriggers.autonomous().whileTrue(Commands.deferredProxy(pathplanner::getSelected));

    /* * * DRIVE BUTTONS * * */
        // reset gyro
        driveJoy.rightBumper()
            .onTrue(drivetrain.resetGyroCmd());

        // speed factor
        driveJoy.leftTrigger()
            .onTrue(drivetrain.slowCmd())
            .onFalse(drivetrain.regularCmd());
        
        // tests led after trigger is triggered -> works!
        driveJoy.a()
            .onTrue(
                // intaking.setIntakeHopperSetpoints(0)
                Commands.waitUntil(hopper::isHopperFull)
                .andThen(leds.setGreenCmd().withTimeout(3))
                // cannot put the withTimeout outside otherwise, it gives it 3 secs for the entier thing)
            );

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
            // entire intake routine
            .onTrue(intaking.moveNote(IntakeHopperConstants.INTAKE_NOTE_SPEED) // bc it's a runOnce, it automatically went to setting sp to 0
                .andThen(Commands.waitUntil(hopper::isHopperFull))
                .andThen(intaking.setIntakeHopperSetpoints(0))
                // .andThen(() -> hopper.resetStateCountCmd()) // testing, commented out before
                .andThen(leds.setGreenCmd().withTimeout(2))
                // .finallyDo(() -> leds.setLedGreen()).withTimeout(2) // need to test to see if andThen or .finallyDo works better
            );

            // unit testing 
            // intaking intakeHopper work
            // .onTrue(intaking.setIntakeHopperSetpoints(3*360))
            // .onFalse(intaking.setIntakeHopperSetpoints(0));
            
            // just intake mech
            // .onTrue(intake.setVelocitySetpointCmd(IntakeConstants.INTAKE_VEL))
            // .onFalse(intake.setVelocitySetpointCmd(0.0));
            
        // runs outtake
        operJoy.leftBumper()
            // entire outtake routine
            .onTrue(intaking.setIntakeHopperSetpoints(-IntakeHopperConstants.INTAKE_NOTE_SPEED))
            .onFalse(intaking.setIntakeHopperSetpoints(0));
            // separate motion
            // .onTrue(intake.setSpeedCmd(-IntakeConstants.ROLLER_SPEED))
            // .onFalse(intake.stopMotorCmd());

            // .onTrue(intake.setVelocitySetpointCmd(IntakeConstants.OUTTAKE_VEL))
            // .onFalse(intake.setVelocitySetpointCmd(0.0));


    /* * * HOPPER BUTTONS * * */
        // runs hopper (towards shooter)
        operJoy.start()
            // .onTrue(hopper.setHopperSpeedCmd(0.7))
            // .onFalse(hopper.stopHopperMotorCmd());
            .onTrue(hopper.setVelocitySetpointCmd(IntakeHopperConstants.INTAKE_NOTE_SPEED))
            .onFalse(hopper.setVelocitySetpointCmd(0));

        // runs reverse hopper (towards intake)
        operJoy.back() // feeds note from hopper to shooter
            .onTrue(Commands.waitUntil(() -> hopper.isHopperFull())
                .andThen(hopper.setVelocitySetpointCmd(HopperConstants.TRANSITION_SPEED))
                .andThen(Commands.waitUntil(hopper::isHopperEmpty))
                .andThen(hopper.setVelocitySetpointCmd(0))
                .andThen(shooterWheel.setVelocitySetpointCmd(0))
                .andThen(hopper.resetStateCountCmd()) // testing, commented in before
                // .finallyDo(() -> leds.setPurpleCmd()).withTimeout(2)
            );

            // TEST resetting state count before setting state count
            // .onTrue(Commands.waitUntil(() -> hopper.isHopperFull())
            //     .andThen(hopper.resetStateCountCmd())
            //     .andThen(hopper.setVelocitySetpointCmd(HopperConstants.TRANSITION_SPEED))
            //     .andThen(Commands.waitUntil(hopper::isHopperEmpty))
            //     .andThen(hopper.setVelocitySetpointCmd(0))
            //     .andThen(shooterWheel.setVelocitySetpointCmd(0))
            //     // .finallyDo(() -> leds.setPurpleCmd()).withTimeout(2)
            // );

            // TEST CALLING CMD
            // .onTrue(hopper.feedNote());

            // FAIL SAFE simple hopper FEED
            // .onTrue(hopper.setVelocitySetpointCmd(HopperConstants.TRANSITION_SPEED))
            // .onFalse(hopper.setVelocitySetpointCmd(0));

    /* * * SHOOTER WHEEL * * */
        // shooting -> positive
        // voltage ramping shooter
        operJoy.rightTrigger()
            .onTrue(hopper.resetStateEmergencyCmd()
        );
            // .onTrue(shooter.shoot(25.0*360));

            // just run shooter
            // .onTrue(shooterWheel.setVelocityCmd(ShooterWheelConstants.AMP_FLUSH));
        
        // // runs shooter intake -> negative
        // // TODO: CHANGE setSpeed to velocity later; FIGURE OUT SHOOTER INTAKE ROUTINE
        operJoy.leftTrigger()
            // reset to default config
            .onTrue(
                shooterAngle.setAngleSetpointCmd(ShooterAngleConstants.SHOOTER_MIN_ANGLE) // amp angle
                // .alongWith(intaking.setIntakeHopperSetpoints(0))
                .alongWith(shooterWheel.setVelocitySetpointCmd(0))
                // .alongWith(hopper.resetStateCountCmd())
            );
            // .onTrue(shooterWheel.setVelocitySetpointCmd(0)
            //     .alongWith(hopper.setVelocitySetpointCmd(0))
            //     .alongWith(shooterWheel.stopMotorsCmd())
            // );
            // shooter intake
            // .onTrue(
            //     new RunCommand(() -> shooterWheel.setSpeed(-0.5), shooterWheel))
            // .onFalse(
            //     new InstantCommand(() -> shooterWheel.stopMotors(), shooterWheel));

    /* * * SHOOTER ANGLE BUTTONS * * */
        // toggles arm manual -> made default command
        // operJoy.rightStick()
        //     .toggleOnTrue(shooterAngle.setManualAngleCmd(
        //         MathUtil.applyDeadband(-operJoy.getRightY(), 0.1)));

        // amp flush
        operJoy.a()
            .onTrue(shooter.setShooterSetpoints(ShooterAngleConstants.AMP_FLUSH, ShooterWheelConstants.AMP_FLUSH)
            .alongWith(hopper.setStateLimitCmd(2))
            );

            // shooting with button cmd
            // .onTrue(shooterWheel.setVelocitySetpointCmd(ShooterAngleConstants.AMP_FLUSH)
                // .andThen(shooter.shoot(ShooterWheelConstants.AMP_FLUSH))
            // );

            // testing at angle 
            // .onTrue(shooterAngle.setAngleSetpointCmd(ShooterAngleConstants.AMP_FLUSH));
        
        // speaker flush
        operJoy.x()
            // .onTrue(shooter.setShooterSetpoints(ShooterAngleConstants.SPEAKER_FLUSH, ShooterWheelConstants.SPEAKER_FLUSH)
            // .alongWith(hopper.setStateLimitCmd(1);
            // );
             .onTrue(shooter.setShooterSetpoints(ShooterAngleConstants.SPEAKER_STAGE, ShooterWheelConstants.SPEAKER_STAGE)
            .alongWith(hopper.setStateLimitCmd(1))
            );
            

            // .onTrue(shooterWheel.setVelocitySetpointCmd(ShooterAngleConstants.SPEAKER_FLUSH));

            
            // shooting with buttons
            // .onTrue(shooterAngle.setAngleSetpointCmd(ShooterAngleConstants.SPEAKER_FLUSH)
            //     .andThen(shooter.shoot(ShooterWheelConstants.SPEAKER_FLUSH))
            // );
            
            // testing at angle 
            // .onTrue(shooterAngle.setAngleSetpointCmd(ShooterAngleConstants.SPEAKER_FLUSH));

        // speaker stage
        // operJoy.y()
        //     .onTrue(shooter.setShooterSetpoints(ShooterAngleConstants.SPEAKER_STAGE, ShooterWheelConstants.SPEAKER_STAGE)
        //     .alongWith(hopper.setStateLimitCmd(1))
        //     );
            // .onTrue(shooterAngle.setAngleSetpointCmd(ShooterAngleConstants.SPEAKER_STAGE)
            //     .alongWith(shooterWheel.setVelocitySetpointCmd(ShooterWheelConstants.SPEAKER_STAGE)));
            
            // testing at angle
            // .onTrue(shooterAngle.setAngleSetpointCmd(ShooterAngleConstants.SPEAKER_STAGE));

        // // speaker wing
        // operJoy.b()
        //     .onTrue(shooter.setShooterSetpoints(ShooterAngleConstants.SPEAKER_STAGE +3 , ShooterWheelConstants.SPEAKER_STAGE));
        //     .onTrue(shooterAngle.setAngleSetpointCmd(ShooterAngleConstants.SPEAKER_WING)
        //         .alongWith(shooterWheel.setVelocitySetpointCmd(ShooterWheelConstants.SPEAKER_WING)));

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
  public SendableChooser<Command> getAutonomousCommand() {
    // Load the path you want to follow using its name in the GUI
    return pathplanner;
    // Create a path following command using AutoBuilder. This will also trigger event markers.
    // List<PathPlannerPath> threeNoteFlush = PathPlannerAuto.getPathGroupFromAutoFile("3 note flush");
    // return pathplannerChooser;
 }
}
