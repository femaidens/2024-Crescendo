// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;

/** Add your docs here. */
public class Intaking {

    private Intake intake;
    private Hopper hopper;
    private LED led;

    public Intaking(Intake intake, Hopper hopper, LED led) {
        this.intake = intake;
        this.hopper = hopper;
        this.led = led;
    }

    public Command intakeNote() {
        return Commands.race(
                led.setSolidCmd(LEDConstants.RED),
                setIntakeHopperSpeeds(0.3) // runOnce
                        .andThen(Commands.waitUntil(hopper::isHopperFull))
                        .andThen(intake.stopMotorCmd().alongWith(hopper.stopMotorCmd()))) // end of race cmd
                .andThen(led.setGreenCmd().withTimeout(2));
    }

    public Command setIntakeHopperSetpoints(double setpoint) {
        return intake.setVelocitySetpointCmd(setpoint)
                .alongWith(hopper.setVelocitySetpointCmd(setpoint));
    }

    public Command setIntakeHopperSpeeds(double speed) {
        return intake.setSpeedCmd(speed)
                .alongWith(hopper.setSpeedCmd(speed));
    }

    public Command setOuttakeSpeeds(double speed) {
        return intake.setOuttakeSpeedCmd(speed)
                .alongWith(hopper.setOuttakeSpeedCmd(speed));
    }
}
