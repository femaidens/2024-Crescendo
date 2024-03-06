// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ShooterAngle;
import frc.robot.subsystems.ShooterWheel;

/** Add your docs here. */
public class Controls {

    private ShooterAngle shooterAngle;
    private ShooterWheel shooterWheel;
    private Hopper hopper;
    private Intake intake;
    private Drivetrain drivetrain;

    public Controls(ShooterAngle shooterAngle, ShooterWheel shooterWheel, Hopper hopper, Intake intake, Drivetrain drivetrain) {
        this.shooterAngle = shooterAngle;
        this.shooterWheel = shooterWheel;
        this.hopper = hopper;
        this.intake = intake;
        this.drivetrain = drivetrain;
    }

    public Command controlSwitch(String controlType, String subsystem, String button) {
        switch(controlType) {
            case "pid" :
                pidSwitch(subsystem, button);
            case "sysid" : 
                sysidSwitch(subsystem, button);
            default :
                return new PrintCommand("no control test running");
            }
    }
    
    public Command pidSwitch(String subsystem, String button) {
        switch(subsystem) {
            case "shooterAngle" :
                switch(button) {
                    case "a" : return shooterAngle.setAngleSetpointCmd(19.9);
                    case "b" : return shooterAngle.setAngleSetpointCmd(25.0);
                    case "x" : return shooterAngle.setAngleSetpointCmd(45.0);
                    case "y" : return shooterAngle.setAngleSetpointCmd(60.0);
                }
            case "shooterWheel" :
                switch(button) {
                    case "a" : return shooterWheel.setVelocitySetpointCmd(0);
                    case "b" : return shooterWheel.setVelocitySetpointCmd(2.0 * 360);
                    case "x" : return shooterWheel.setVelocitySetpointCmd(5.0 * 360);
                    case "y" : return shooterWheel.setVelocitySetpointCmd(10.0 * 360);
                }
            default : return new PrintCommand("");
        }
    }

    public Command sysidSwitch(String subsystem, String button) {
        switch(subsystem) {
            case "hopper" :
                switch(button) {
                    case "a" : return hopper.hopperQuas(SysIdRoutine.Direction.kForward);
                    case "b" : return hopper.hopperQuas(SysIdRoutine.Direction.kReverse);
                    case "x" : return hopper.hopperDyna(SysIdRoutine.Direction.kForward);
                    case "y" : return hopper.hopperDyna(SysIdRoutine.Direction.kReverse);
                }
            case "intake" :
                switch(button) {
                    case "a" : return intake.intakeQuas(SysIdRoutine.Direction.kForward);
                    case "b" : return intake.intakeQuas(SysIdRoutine.Direction.kReverse);
                    case "x" : return intake.intakeDyna(SysIdRoutine.Direction.kForward);
                    case "y" : return intake.intakeDyna(SysIdRoutine.Direction.kReverse);
                }
            case "drivetrain" :
                switch(button) {
                    case "a" : return drivetrain.driveQuasistatic(SysIdRoutine.Direction.kForward);
                    case "b" : return drivetrain.driveQuasistatic(SysIdRoutine.Direction.kReverse);
                    case "x" : return drivetrain.driveDynamic(SysIdRoutine.Direction.kForward);
                    case "y" : return drivetrain.driveDynamic(SysIdRoutine.Direction.kReverse);
                    case "rightBumper" : return drivetrain.turnQuasistatic(SysIdRoutine.Direction.kForward); 
                    case "leftBumper" : return drivetrain.turnDynamic(SysIdRoutine.Direction.kForward);
                }
            default : return new PrintCommand("");
        }
    }
}
