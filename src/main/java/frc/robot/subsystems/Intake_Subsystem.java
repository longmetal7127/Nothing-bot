package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.sim.SparkFlexSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VortexConstants;

/** Controls the NEO Vortex through its docked SPARK Flex controller. */
@Logged
public class Intake_Subsystem extends SubsystemBase {
  private final SparkFlex vortexMotor = new SparkFlex(VortexConstants.VORTEX_MOTOR_ID, MotorType.kBrushless);
  private static final DCMotor VORTEX_GEARBOX = DCMotor.getNeoVortex(1);
  private final SparkFlexSim vortexMotorSim = new SparkFlexSim(vortexMotor, VORTEX_GEARBOX);
  // Approximate inertia for a free-spinning roller; adjust if you know the mechanism inertia.
  private final DCMotorSim vortexPhysicsSim = new DCMotorSim(
      LinearSystemId.createDCMotorSystem(VORTEX_GEARBOX, 0.0005, 1.0),
      VORTEX_GEARBOX);

  /** Sets motor output from -1.0 (reverse) to 1.0 (forward). */
  public void setOutput(double output) {
    vortexMotor.set(output);
  }

  /** Stops the motor. */
  public void stop() {
    vortexMotor.stopMotor();
  }

  /** Motor output fraction, from -1.0 to 1.0; logged by Epilogue. */
  public double getAppliedOutput() {
    return vortexMotor.getAppliedOutput();
  }

  /** Motor current in amps; logged by Epilogue. */
  public double getCurrentAmps() {
    return vortexMotor.getOutputCurrent();
  }

  /** Simulated motor speed in RPM; useful in simulation telemetry. */
  public double getSimulatedRpm() {
    return vortexPhysicsSim.getAngularVelocityRPM();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Vortex/Applied Output", vortexMotor.getAppliedOutput());
    SmartDashboard.putNumber("Vortex/Current (A)", vortexMotor.getOutputCurrent());
  }

  @Override
  public void simulationPeriodic() {
    double batteryVoltage = RoboRioSim.getVInVoltage();

    vortexPhysicsSim.setInputVoltage(vortexMotorSim.getAppliedOutput() * batteryVoltage);
    vortexPhysicsSim.update(0.02);

    vortexMotorSim.iterate(vortexPhysicsSim.getAngularVelocityRPM(), batteryVoltage, 0.02);
    SmartDashboard.putNumber("Vortex/Sim RPM", vortexPhysicsSim.getAngularVelocityRPM());
  }
}
