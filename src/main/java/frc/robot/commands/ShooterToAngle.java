// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import frc.robot.subsystems.ShooterAngle;
// import frc.robot.subsystems.ShooterWheel;

// public class ShooterToAngle extends Command {
    
//     private final ShooterAngle shooterAngle;
//     private final double angle;

//     public ShooterToAngle(ShooterAngle shooterAngle, double angle) {
//         this.shooterAngle = shooterAngle;
//         this.angle = angle;
        
//         addRequirements(shooterAngle);
//     }

//     @Override
//     public void initialize() {
//         shooterAngle.setAngle(angle);
//     }

//     @Override
//     public boolean isFinished() {
//         return shooterAngle.atAngle();
//     }
// }
