// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ShooterConstants;
import frc.robot.Ports.*;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterAngle;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Shooter m_shooter = new Shooter();
  private final ShooterAngle m_shooterAngle = new ShooterAngle();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  // private final CommandXboxController driveJoy =
  //     new CommandXboxController(JoystickPorts.OPER_JOY);
  private CommandXboxController m_driverController = new CommandXboxController(Ports.JoystickPorts.DRIVE_JOY);
  private CommandXboxController m_operController = new CommandXboxController(Ports.JoystickPorts.OPER_JOY);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    //m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

    //driveJoy.a().whileTrue(climber.extendClimbArm());
    Trigger shooterSpin = m_operController.a();
      shooterSpin
        .onTrue(new RunCommand(
          () -> m_shooter.setDesiredVelocity(ShooterConstants.SHOOTER_METERS_SECOND), m_shooter))
        .onFalse(new RunCommand(
          () -> m_shooter.stopShooter(), m_shooter));

    Trigger shooterUp = m_operController.b();
      shooterUp
        .onTrue(new RunCommand(
          () -> m_shooterAngle.shooterAngleUp(), m_shooterAngle
        ))
        .onFalse(new RunCommand(
          () -> m_shooterAngle.shooterAngleStop(), m_shooterAngle));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
}
