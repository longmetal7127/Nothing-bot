// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.sim.SparkMaxSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.FuelConstants.*;

@Logged
public class CANFuelSubsystem extends SubsystemBase {
  private final SparkMax LeftIntakeLauncher;
  private final SparkMax RightIntakeLauncher;

  private final SparkMaxSim leftLauncherSim;
  private final SparkMaxSim rightLauncherSim;
  private final DCMotorSim leftLauncherPhysicsSim;
  private final DCMotorSim rightLauncherPhysicsSim;

  /** Creates a new CANBallSubsystem. */
  public CANFuelSubsystem() {
    // create brushed motors for each of the motors on the launcher mechanism
    LeftIntakeLauncher = new SparkMax(LEFT_INTAKE_LAUNCHER_MOTOR_ID, MotorType.kBrushless);
    RightIntakeLauncher = new SparkMax(RIGHT_INTAKE_LAUNCHER_MOTOR_ID, MotorType.kBrushless);

    // Simulate the two brushless NEO shooter motors.
    DCMotor shooterMotorModel = DCMotor.getNEO(1);
    leftLauncherSim = new SparkMaxSim(LeftIntakeLauncher, shooterMotorModel);
    rightLauncherSim = new SparkMaxSim(RightIntakeLauncher, shooterMotorModel);
    leftLauncherPhysicsSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(shooterMotorModel, 0.0005, 1.0), shooterMotorModel);
    rightLauncherPhysicsSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(shooterMotorModel, 0.0005, 1.0), shooterMotorModel);

    // create the configuration for the launcher roller, set a current limit, set
    // the motor to inverted so that positive values are used for both intaking and
    // launching, and apply the config to the controller
    // CAN 5 is the reference direction. CAN 6 is inverted so its shooter motor
    // spins opposite CAN 5. Separate configs prevent inversion carrying over.
    SparkMaxConfig can5LauncherConfig = new SparkMaxConfig();
    can5LauncherConfig.smartCurrentLimit(LAUNCHER_MOTOR_CURRENT_LIMIT);
    can5LauncherConfig.voltageCompensation(12);
    can5LauncherConfig.idleMode(IdleMode.kCoast);
    LeftIntakeLauncher.configure(
        can5LauncherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig can6LauncherConfig = new SparkMaxConfig();
    can6LauncherConfig.smartCurrentLimit(LAUNCHER_MOTOR_CURRENT_LIMIT);
    can6LauncherConfig.voltageCompensation(12);
    can6LauncherConfig.idleMode(IdleMode.kCoast);
    can6LauncherConfig.inverted(true);
    RightIntakeLauncher.configure(
        can6LauncherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // put default values for various fuel operations onto the dashboard
    // all commands using this subsystem pull values from the dashbaord to allow
    // you to tune the values easily, and then replace the values in Constants.java
    // with your new values. For more information, see the Software Guide.
    SmartDashboard.putNumber("Intaking intake roller value", INTAKE_INTAKING_PERCENT);
    SmartDashboard.putNumber("Launching launcher roller value", LAUNCHING_LAUNCHER_PERCENT);
    //SmartDashboard.putNumber("Spin-up feeder roller value", SPIN_UP_FEEDER_VOLTAGE);
  }

  // A method to set the voltage of the intake roller
  public void setIntakeLauncherRoller(double power) {
    LeftIntakeLauncher.set(power);
    RightIntakeLauncher.set(power); // positive for shooting
  }

  // A method to set the voltage of the intake roller
  public void setFeederRoller(double power) {
    // CAN 8 indexer was removed. Kept as a no-op so existing commands still compile.
  }

  // A method to stop the rollers
  public void stop() {
    LeftIntakeLauncher.set(0);
    RightIntakeLauncher.set(0);
  }

  /** Left shooter motor output fraction, from -1.0 to 1.0. */
  public double getLeftShooterOutput() {
    return LeftIntakeLauncher.getAppliedOutput();
  }

  /** Right shooter motor output fraction, from -1.0 to 1.0. */
  public double getRightShooterOutput() {
    return RightIntakeLauncher.getAppliedOutput();
  }

  /** Left shooter speed from the desktop motor simulation, in RPM. */
  public double getLeftShooterSimRpm() {
    return leftLauncherPhysicsSim.getAngularVelocityRPM();
  }

  /** Right shooter speed from the desktop motor simulation, in RPM. */
  public double getRightShooterSimRpm() {
    return rightLauncherPhysicsSim.getAngularVelocityRPM();
  }

  @Override
  public void periodic() {
    // Show live motor output and current on the SmartDashboard. Applied output
    // is a fraction from -1.0 to 1.0; current is measured in amps.
    SmartDashboard.putNumber("Fuel/Left Shooter Output", LeftIntakeLauncher.getAppliedOutput());
    SmartDashboard.putNumber("Fuel/Left Shooter Current (A)", LeftIntakeLauncher.getOutputCurrent());
    SmartDashboard.putNumber("Fuel/Right Shooter Output", RightIntakeLauncher.getAppliedOutput());
    SmartDashboard.putNumber("Fuel/Right Shooter Current (A)", RightIntakeLauncher.getOutputCurrent());
  }

  @Override
  public void simulationPeriodic() {
    double batteryVoltage = RoboRioSim.getVInVoltage();
    updateMotorSimulation(leftLauncherSim, leftLauncherPhysicsSim, batteryVoltage, "Fuel/Left Shooter Sim RPM");
    updateMotorSimulation(rightLauncherSim, rightLauncherPhysicsSim, batteryVoltage, "Fuel/Right Shooter Sim RPM");
  }

  private void updateMotorSimulation(
      SparkMaxSim motorSim, DCMotorSim physicsSim, double batteryVoltage, String rpmKey) {
    physicsSim.setInputVoltage(motorSim.getAppliedOutput() * batteryVoltage);
    physicsSim.update(0.02);
    motorSim.iterate(physicsSim.getAngularVelocityRPM(), batteryVoltage, 0.02);
    SmartDashboard.putNumber(rpmKey, physicsSim.getAngularVelocityRPM());
  }
}
