// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

    public Command shoot(double vel) {
        // return Commands.waitUntil(() -> shooterWheel.atVelocity() && hopper.isHopperFull())
        //     .andThen(hopper.feedNote())
        //     .deadlineWith(shooterWheel.setVelocityCmd(vel))
        //     .finallyDo(() -> {
        //         shooterWheel.setVelocitySetpointCmd(0);
        //         hopper.setVelocitySetpointCmd(0);
        //         shooterWheel.stopMotors();
        //         hopper.stopMotor();
        //         }
        //     );
        // return Commands.waitUntil(() -> shooterWheel.atVelocity() && hopper.isHopperFull())
        // .andThen(hopper.feedNote())
        // .andThen(shooterWheel.setVelocitySetpointCmd(vel))
        // .deadlineWith(shooterWheel.setVelocityCmd(10*360.0));

        return Commands.waitUntil(() -> shooterWheel.atVelocity() && shooterAngle.atAngle())
            .andThen(hopper.feedNote())
            // .andThen(Commands.waitUntil(hopper::isHopperEmpty))
            .andThen(shooterWheel.setVelocitySetpointCmd(0)
        );
    }

    public Command shoot() {

        return Commands.waitUntil(() -> shooterWheel.atVelocity() && shooterAngle.atAngle())
            .andThen(hopper.feedNote())
            // .andThen(Commands.waitUntil(hopper::isHopperEmpty))
            .andThen(shooterWheel.setVelocitySetpointCmd(0)
        );
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
