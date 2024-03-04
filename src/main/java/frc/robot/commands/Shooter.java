// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.Constants.ShooterAngleConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ShooterAngle;
import frc.robot.subsystems.ShooterWheel;

/** Add your docs here. */
public class Shooter {

    private final ShooterAngle shooterAngle;
    private final ShooterWheel shooterWheel;
    private final Intake intake;

    public Shooter(ShooterAngle shooterAngle, ShooterWheel shooterWheel, Intake intake) {
        this.shooterAngle = shooterAngle;
        this.shooterWheel = shooterWheel;
        this.intake = intake;
    }

    public Command shoot() {
        return new ConditionalCommand(
            (shooterWheel.setVelocityCmd().until(shooterWheel::atVelocity)) // ramps shooter to desired velocity
                .andThen(intake.setHopperVelocityCmd()), // moves hopper after desired vel is reached
            (shooterWheel.stopMotorsCmd().alongWith(intake.stopHopperMotorCmd())), // stops motors first
            intake::isHopperEmpty // stops first command when hopper is ready
        );
    }

    public Command autonShoot(double angle) {
        return shoot().beforeStarting(shooterAngle.SetAngleSetpointCmd(angle));
    }

}
