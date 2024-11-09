// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.modules;

import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.DrivetrainConstants;

/** Add your docs here. */
public class SimModule implements ModuleIO {
    private final DCMotorSim drive =
      new DCMotorSim(LinearSystemId.createDCMotorSystem(
        DrivetrainConstants.ModuleConstants.Drive.kP, DrivetrainConstants.ModuleConstants.Drive.kI)
        DCMotor.getNeoVortex(1)
        );
}
