// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.ShooterAngle;
import frc.robot.subsystems.ShooterWheel;

/** Add your docs here. */
public class Shooter {

    private final ShooterAngle shooterAngle;
    private final ShooterWheel shooterWheel;
    private final Hopper hopper;
    private final LED led;

    public Shooter(ShooterAngle shooterAngle, ShooterWheel shooterWheel, Hopper hopper, LED led) {
        this.shooterAngle = shooterAngle;
        this.shooterWheel = shooterWheel;
        this.hopper = hopper;
        this.led = led;
    }

    public Command shoot(double vel) { // figure out what to do with the velocity param -> is it necessary?

        // CHECKING IS SHOOTER AT ANGLE
        return Commands.waitUntil(() -> shooterWheel.atVelocity() && shooterAngle.atAngle())
                .andThen(hopper.feedNote())
                .andThen(shooterWheel.setVelocitySetpointCmd(0))
                .andThen(hopper.resetStateCountCmd())
                .andThen(led.setSolidCmd(LEDConstants.PURPLE).withTimeout(2));
    }

    public Command shoot() {
        return Commands.waitUntil(() -> shooterWheel.atVelocity() && shooterAngle.atAngle())
                .andThen(hopper.feedNote())
                .andThen(shooterWheel.setVelocitySetpointCmd(0))
                .andThen(hopper.resetStateCountCmd())
                .andThen(led.setSolidCmd(LEDConstants.PURPLE).withTimeout(2));
    }

    public Command setShooterSetpoints(double angle, double vel) {
        return shooterAngle.setAngleSetpointCmd(angle)
                .alongWith(shooterWheel.setVelocitySetpointCmd(vel));
    }

    public Command autonShoot(double vel, double angle) {
        return shoot().beforeStarting(shooterAngle.setAngleSetpointCmd(angle)); // double check which shoot cmd to call
    }

    public Command resetAutonSetpoints() {
        return shooterAngle.setAngleSetpointCmd(0)
                .alongWith(hopper.setVelocitySetpointCmd(0))
                .alongWith(shooterWheel.setVelocitySetpointCmd(0));
    }

}
