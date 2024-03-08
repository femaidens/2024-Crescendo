// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.IntakeHopperConstants;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;

/** Add your docs here. */
public class Intaking {
    
    private Intake intake;
    private Hopper hopper;

    public Intaking(Intake intake, Hopper hopper) {
        this.intake = intake;
        this.hopper = hopper;
    }

    public Command moveNote(double velocity) {
        return intake.setVelocitySetpointCmd(velocity)
                .alongWith(hopper.setVelocitySetpointCmd(velocity));
        // return Commands.waitUntil(hopper::isHopperEmpty)
        // .andThen(Commands.waitUntil(hopper::isHopperFull)
        // .deadlineWith(setIntakeHopperSetpoints(velocity)));
    }

    public Command setIntakeHopperSetpoints(double setpoint) {
        return intake.setVelocitySetpointCmd(setpoint)
        .alongWith(hopper.setVelocitySetpointCmd(setpoint));
    }
}
