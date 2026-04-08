package frc.robot.subsystems.Shooter;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {

  private final SparkMax feederMotor = new SparkMax(55, MotorType.kBrushless);
  private final SparkMax feederMotor1 = new SparkMax(0, MotorType.kBrushless);

  // Positive = feed into shooter
  // Negative = outtake away from shooter (into hopper)

  public Command feed() {
    return this.runOnce(() -> feederMotor.set(-0.85));
  }
  public Command feed1() {
    return this.runOnce(() -> feederMotor1.set(-0.85));
  }
  public Command stop() {
    return this.runOnce(() -> feederMotor.set(0));
  }
  public Command stop1() {
    return this.runOnce(() -> feederMotor1.set(0));
  }
  public double getSpeed() {
    return feederMotor.get();
  }
}
