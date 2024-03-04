// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.Constants.ShooterAngleConstants;
import frc.robot.Constants.ShooterWheelConstants;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.ShooterAngle;
import frc.robot.subsystems.ShooterWheel;

/** Add your docs here. */
public class Shooter {

    private final ShooterAngle shooterAngle;
    private final ShooterWheel shooterWheel;
    private final Hopper hopper;

    public Shooter(ShooterAngle shooterAngle, ShooterWheel shooterWheel, Hopper hopper) {
        this.shooterAngle = shooterAngle;
        this.shooterWheel = shooterWheel;
        this.hopper = hopper;
    }

    public Command shoot() {
        return new ConditionalCommand(
            (shooterWheel.setVelocityCmd().until(shooterWheel::atVelocity)) // ramps shooter to desired velocity
                .andThen(hopper.setHopperVelocityCmd()), // moves hopper after desired vel is reached
            (shooterWheel.stopMotorsCmd().alongWith(hopper.stopHopperMotorCmd())), // stops motors first
            hopper::isHopperEmpty // stops first command when hopper is ready
        );
    }

    public Command setShooterSetpoints(double angle, double vel) {
        return shooterAngle.setAngleSetpointCmd(angle)
                .alongWith(shooterWheel.setVelocitySetpointCmd(vel));
    }

    public Command autonShoot(double angle) {
        return shoot().beforeStarting(shooterAngle.setAngleSetpointCmd(angle));
    }

}
