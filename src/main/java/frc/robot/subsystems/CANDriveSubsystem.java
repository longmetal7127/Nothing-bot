// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.DriveConstants.*;
@Logged
public class CANDriveSubsystem extends SubsystemBase {
  private final SparkMax leftLeader;
  private final SparkMax leftFollower;
  private final SparkMax rightLeader;
  private final SparkMax rightFollower;

  private final DifferentialDrive drive;

  // Desktop sim needs a motor model to produce changing encoder values.
  // These motors are configured as brushed; CIM is the assumed motor type.
  private static final DCMotor DRIVE_MOTOR_MODEL = DCMotor.getCIM(1);
  private final SparkMaxSim leftLeaderSim;
  private final SparkMaxSim leftFollowerSim;
  private final SparkMaxSim rightLeaderSim;
  private final SparkMaxSim rightFollowerSim;
  private final DCMotorSim leftLeaderPhysicsSim = createDriveMotorSim();
  private final DCMotorSim leftFollowerPhysicsSim = createDriveMotorSim();
  private final DCMotorSim rightLeaderPhysicsSim = createDriveMotorSim();
  private final DCMotorSim rightFollowerPhysicsSim = createDriveMotorSim();

  private static DCMotorSim createDriveMotorSim() {
    // This estimates each free-spinning motor shaft. Gearbox/wheel dynamics can
    // be added later if you want a full robot-motion simulation.
    return new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DRIVE_MOTOR_MODEL, 0.001, 1.0),
        DRIVE_MOTOR_MODEL);
  }

  public CANDriveSubsystem() {
    // create brushed motors for drive
    leftLeader = new SparkMax(LEFT_LEADER_ID, MotorType.kBrushed);
    leftFollower = new SparkMax(LEFT_FOLLOWER_ID, MotorType.kBrushed);
    rightLeader = new SparkMax(RIGHT_LEADER_ID, MotorType.kBrushed);
    rightFollower = new SparkMax(RIGHT_FOLLOWER_ID, MotorType.kBrushed);

    leftLeaderSim = new SparkMaxSim(leftLeader, DRIVE_MOTOR_MODEL);
    leftFollowerSim = new SparkMaxSim(leftFollower, DRIVE_MOTOR_MODEL);
    rightLeaderSim = new SparkMaxSim(rightLeader, DRIVE_MOTOR_MODEL);
    rightFollowerSim = new SparkMaxSim(rightFollower, DRIVE_MOTOR_MODEL);

    // set up differential drive class
    drive = new DifferentialDrive(leftLeader, rightLeader);

    // Set can timeout. Because this project only sets parameters once on
    // construction, the timeout can be long without blocking robot operation. Code
    // which sets or gets parameters during operation may need a shorter timeout.
    leftLeader.setCANTimeout(250);
    rightLeader.setCANTimeout(250);
    leftFollower.setCANTimeout(250);
    rightFollower.setCANTimeout(250);

    // Create the configuration to apply to motors. Voltage compensation
    // helps the robot perform more similarly on different
    // battery voltages (at the cost of a little bit of top speed on a fully charged
    // battery). The current limit helps prevent tripping
    // breakers.
    SparkMaxConfig config = new SparkMaxConfig();
    config.voltageCompensation(12);
    config.smartCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT);
    config.idleMode(IdleMode.kBrake);

    // Set configuration to follow each leader and then apply it to corresponding
    // follower. Resetting in case a new controller is swapped
    // in and persisting in case of a controller reset due to breaker trip
    config.follow(leftLeader);
    leftFollower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    config.follow(rightLeader);
    rightFollower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    

    // Remove following, then apply config to right leader
    config.disableFollowerMode();
    rightLeader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // Set config to inverted and then apply to left leader. Set Left side inverted
    // so that postive values drive both sides forward
    config.inverted(true);
    leftLeader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
    double batteryVoltage = RoboRioSim.getVInVoltage();
    double leftOutput = leftLeaderSim.getAppliedOutput();
    double rightOutput = rightLeaderSim.getAppliedOutput();
    updateMotorSimulation(leftLeaderSim, leftLeaderPhysicsSim, leftOutput, batteryVoltage,
        "Drive/Left Leader Sim RPM", "Drive/Left Leader Sim Rotations");
    updateMotorSimulation(leftFollowerSim, leftFollowerPhysicsSim, leftOutput, batteryVoltage,
        "Drive/Left Follower Sim RPM", "Drive/Left Follower Sim Rotations");
    updateMotorSimulation(rightLeaderSim, rightLeaderPhysicsSim, rightOutput, batteryVoltage,
        "Drive/Right Leader Sim RPM", "Drive/Right Leader Sim Rotations");
    updateMotorSimulation(rightFollowerSim, rightFollowerPhysicsSim, rightOutput, batteryVoltage,
        "Drive/Right Follower Sim RPM", "Drive/Right Follower Sim Rotations");
  }

  private void updateMotorSimulation(
      SparkMaxSim motorSim, DCMotorSim physicsSim, double appliedOutput, double batteryVoltage,
      String rpmKey, String rotationsKey) {
    // Followers do not receive independent commands, so their simulated motor
    // input comes from the leader on that side of the drivetrain.
    physicsSim.setInputVoltage(appliedOutput * batteryVoltage);
    physicsSim.update(0.02);
    motorSim.iterate(physicsSim.getAngularVelocityRPM(), batteryVoltage, 0.02);
    SmartDashboard.putNumber(rpmKey, physicsSim.getAngularVelocityRPM());
    SmartDashboard.putNumber(rotationsKey, physicsSim.getAngularPositionRotations());
  }

  public void driveArcade(double xSpeed, double zRotation) {
    drive.arcadeDrive(xSpeed, zRotation);
  }

}
