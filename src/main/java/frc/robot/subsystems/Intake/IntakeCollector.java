package frc.robot.subsystems.Intake;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeCollector extends SubsystemBase {

  private final SparkMax collectorMotor = new SparkMax(9, MotorType.kBrushless);

  /**
   * Spin the motor at a given speed. Positive is intake, negative is outtake.
   * 
   * @param speed The speed to spin the motor at, between -1 and 1.
   * @return A command that spins the motor at the given speed.
   */
  public Command spin(double speed) {
    return this.runOnce(() -> collectorMotor.set(speed));
  }

  public Command stop() {
    return this.runOnce(() -> collectorMotor.set(0));
  }

  public Command run() {
    return this.startEnd(
      () -> collectorMotor.set(0.5),
      () -> collectorMotor.set(0)
    );
  }
}
