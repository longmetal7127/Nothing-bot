package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VortexConstants;

/** Controls the NEO Vortex through its docked SPARK Flex controller. */
public class VortexSubsystem extends SubsystemBase {
  private final SparkFlex vortexMotor = new SparkFlex(VortexConstants.VORTEX_MOTOR_ID, MotorType.kBrushless);

  /** Sets motor output from -1.0 (reverse) to 1.0 (forward). */
  public void setOutput(double output) {
    vortexMotor.set(output);
  }

  /** Stops the motor. */
  public void stop() {
    vortexMotor.stopMotor();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Vortex/Applied Output", vortexMotor.getAppliedOutput().get(0.0));
    SmartDashboard.putNumber("Vortex/Current (A)", vortexMotor.getOutputCurrent().get(0.0));
  }
}
