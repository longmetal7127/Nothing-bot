// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VortexConstants;

/** Controls the NEO Vortex intake using its SPARK Flex controller. */
public class VortexSubsystem extends SubsystemBase {
  private final SparkFlex vortexMotor =
      new SparkFlex(VortexConstants.VORTEX_MOTOR_ID, MotorType.kBrushless);

  /** Sets motor output from -1.0 (reverse) to 1.0 (forward). */
  public void setOutput(double output) {
    vortexMotor.set(output);
  }

  /** Stops the motor. */
  public void stop() {
    vortexMotor.stopMotor();
  }
}
