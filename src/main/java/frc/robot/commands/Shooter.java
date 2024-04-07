// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.ShooterAngleConstants;
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

    public Command shoot(double angle, double vel) { // figure out what to do with the velocity param -> is it necessary?
        return setShooterSetpoints(angle, vel)
                .andThen(shoot());
    }

    public Command shoot() {
        // TEST ACCURACY OF SHOOTER AT ANGLE
        return Commands.race(led.setSolidCmd(LEDConstants.RED),
                Commands.waitUntil(() -> shooterAngle.atAngle()) // shooterWheel.atVelocity() &&
                        .andThen(hopper.feedNote(HopperConstants.TRANSITION_VEL))
                // hopper.feedNote()
                        .andThen(shooterWheel.setVelocitySetpointCmd(0))
                        .andThen(hopper.resetStateCountCmd())) // end of race cmd
                .andThen(led.setSolidCmd(LEDConstants.GREEN).withTimeout(3));
    }

    public Command setShooterSetpoints(double angle, double vel) {
        return shooterAngle.setAngleSetpointCmd(angle)
                .alongWith(shooterWheel.setVelocitySetpointCmd(vel));
    }

    // public Command autonShoot(double vel, double angle) {
       
    // }

    public Command resetAutonSetpoints() {
        return shooterAngle.setAngleSetpointCmd(0)
                .alongWith(hopper.setVelocitySetpointCmd(0))
                .alongWith(shooterWheel.setVelocitySetpointCmd(0));
    }

}
