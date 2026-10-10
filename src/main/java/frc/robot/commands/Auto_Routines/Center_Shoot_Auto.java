// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto_Routines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Bot_commands.LaunchSequence;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CANFuelSubsystem;

public class Center_Shoot_Auto extends SequentialCommandGroup {
  public Center_Shoot_Auto(CANDriveSubsystem driveSubsystem, CANFuelSubsystem ballSubsystem) {
    addCommands(
    // Drive backwards for 3 seconds. The driveArcadeAuto command factory
    // intentionally creates a command which does not end which allows us to control
    // the timing using the withTimeout decorator 

    new AutoDrive(driveSubsystem,-0.5,  0.0).withTimeout(3));
    new LaunchSequence(ballSubsystem).withTimeout(10);

    
    
  }
}
