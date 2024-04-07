package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.ShooterAngle;
import frc.robot.subsystems.ShooterWheel;

public class ShooterSetRPM extends InstantCommand {
    
    private final ShooterWheel shooterWheel;
    private final Hopper hopper;
    private final double rpm;
    private final int stateCount;

    public ShooterSetRPM(ShooterWheel shooterWheel, Hopper hopper, double rpm, int stateCount) {
        this.shooterWheel = shooterWheel;
        this.hopper = hopper;
        this.rpm = rpm;
        this.stateCount = stateCount;
        
        addRequirements(shooterWheel, hopper);
    }

    @Override
    public void initialize() {
        shooterWheel.setSpeed(rpm);
        hopper.setStateLimit(stateCount);
    }
}
