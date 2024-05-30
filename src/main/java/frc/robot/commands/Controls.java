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

    public enum ControlType {
        PID,
        SYSID
    }

    public enum Subsystem {
        SHOOTER_ANGLE,
        SHOOTER_WHEEL,
        HOPPER,
        INTAKE,
        DRIVETRAIN
    }

    public enum Button {
        A, 
        B, 
        X, 
        Y,
        RIGHT_BUMPER,
        LEFT_BUMPER
    }

    public Controls(ShooterAngle shooterAngle, ShooterWheel shooterWheel, Hopper hopper, Intake intake, Drivetrain drivetrain) {
        this.shooterAngle = shooterAngle;
        this.shooterWheel = shooterWheel;
        this.hopper = hopper;
        this.intake = intake;
        this.drivetrain = drivetrain;
    }
    /**
     * @param controlType - PID, SYSID
     * @param subsystem - [PID: SHOOTER_ANGLE, SHOOTER_WHEEL] [SYSID: HOPPER, INTAKE, DRIVETRAIN]
     * @param button A, B, X, Y, RIGHT_BUMPER, LEFT_BUMPER
    */
    public Command controlSwitch(ControlType controlType, Subsystem subsystem, Button button) {
        switch(controlType) {
            case PID :
                return pidSwitch(subsystem, button);
            case SYSID : 
                return sysidSwitch(subsystem, button);
            default :
                return new PrintCommand("no control test running");
            }
    }
    
    public Command pidSwitch(Subsystem subsystem, Button button) {
        switch(subsystem) {
            case SHOOTER_ANGLE :
                switch(button) {
                    case A : return shooterAngle.setAngleSetpointCmd(19.9);
                    case B : return shooterAngle.setAngleSetpointCmd(25.0);
                    case X : return shooterAngle.setAngleSetpointCmd(45.0);
                    case Y : return shooterAngle.setAngleSetpointCmd(60.0);
                    default : return new PrintCommand("invalid control"); 
                }
            case SHOOTER_WHEEL :
                switch(button) {
                    case A : return shooterWheel.setVelocitySetpointCmd(0);
                    case B : return shooterWheel.setVelocitySetpointCmd(2.0 * 360);
                    case X : return shooterWheel.setVelocitySetpointCmd(5.0 * 360);
                    case Y : return shooterWheel.setVelocitySetpointCmd(10.0 * 360);
                    default : return new PrintCommand("invalid control");
                }
            default : return new PrintCommand("");
        }
    }

    public Command sysidSwitch(Subsystem subsystem, Button button) {
        switch(subsystem) {
            case HOPPER :
                switch(button) {
                    case A : return hopper.hopperQuas(SysIdRoutine.Direction.kForward);
                    case B : return hopper.hopperQuas(SysIdRoutine.Direction.kReverse);
                    case X : return hopper.hopperDyna(SysIdRoutine.Direction.kForward);
                    case Y : return hopper.hopperDyna(SysIdRoutine.Direction.kReverse);
                    default : return new PrintCommand("invalid control");
                }
            case INTAKE :
                switch(button) {
                    case A : return intake.intakeQuas(SysIdRoutine.Direction.kForward);
                    case B : return intake.intakeQuas(SysIdRoutine.Direction.kReverse);
                    case X : return intake.intakeDyna(SysIdRoutine.Direction.kForward);
                    case Y : return intake.intakeDyna(SysIdRoutine.Direction.kReverse);
                    default : return new PrintCommand("invalid control");
                }
            case DRIVETRAIN :
                switch(button) {
                    case A : return drivetrain.driveQuasistatic(SysIdRoutine.Direction.kForward);
                    case B : return drivetrain.driveQuasistatic(SysIdRoutine.Direction.kReverse);
                    case X : return drivetrain.driveDynamic(SysIdRoutine.Direction.kForward);
                    case Y : return drivetrain.driveDynamic(SysIdRoutine.Direction.kReverse);
                    case RIGHT_BUMPER : return drivetrain.turnQuasistatic(SysIdRoutine.Direction.kForward); 
                    case LEFT_BUMPER : return drivetrain.turnDynamic(SysIdRoutine.Direction.kForward);
                }
            default : return new PrintCommand("");
        }
    }
}
