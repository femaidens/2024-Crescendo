// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Constants.HopperConstants;
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

    //comment out default shooterangle and shooterwheel beforehand
    public Command modifiedShoot(double angle, double vel) { // figure out what to do with the velocity param -> is it necessary?
        return setShooterSetpoints(angle, vel)
                .andThen(modifiedShoot());
    }
    public Command modifiedShoot(){
        return Commands.race(
                led.setSolidCmd(LEDConstants.RED),
                shooterAngle.setAngleCmd().alongWith(shooterWheel.setVelocityCmd()).until(shooterAngle::atAngle) //.withTimeout(3)
                        .andThen(hopper.feedNote(HopperConstants.TRANSITION_VEL))
                        .andThen(shooterWheel.setVelocityCmd(0)) //.andThen(shooterWheel.run(() -> shooterWheel.stopMotors()))
                        .andThen(hopper.resetStateCountCmd())
                )
                .andThen(led.setSolidCmd(LEDConstants.GREEN).withTimeout(3));
    }

    public Command setShooterSetpoints(double angle, double vel) {
        return shooterAngle.setAngleSetpointCmd(angle)
                .alongWith(shooterWheel.setVelocitySetpointCmd(vel));
    }

    // public Command shoot(double angle, double vel) { // figure out what to do with the velocity param -> is it
    //                                                  // necessary?
    //     return setShooterSetpoints(angle, vel)
    //         .alongWith(new PrintCommand("set angle"))
    //         .andThen(shoot());
    // }

    // public Command shoot() {
    //     // TEST ACCURACY OF SHOOTER AT ANGLE
    //     return Commands.race(led.setSolidCmd(LEDConstants.RED),
    //             Commands.waitUntil(() -> shooterAngle.atAngle()) // shooterWheel.atVelocity() &&
    //                     .andThen(new PrintCommand("angle reached"))
    //                     .andThen(hopper.feedNote())
    //                     .andThen(shooterWheel.setVelocitySetpointCmd(0))
    //                     .andThen(hopper.resetStateCountCmd())) // end of race cmd
    //             .andThen(led.setSolidCmd(LEDConstants.GREEN)).withTimeout(3);
    // }

    // public Command setShooterSetpoints(double angle, double vel) {
    //     return shooterAngle.setAngleSetpointCmd(angle)
    //             .alongWith(shooterWheel.setVelocitySetpointCmd(vel));
    // }

    public Command autonShoot(double vel, double angle) {
        return shoot().beforeStarting(shooterAngle.setAngleSetpointCmd(angle)); // double check which shoot cmd to call
    }

    public Command resetAutonSetpoints() {
        return shooterAngle.setAngleSetpointCmd(0)
                .alongWith(hopper.setVelocitySetpointCmd(0))
                .alongWith(shooterWheel.setVelocitySetpointCmd(0));
    }

}
