package frc.robot.subsystems.Shooter;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {

  private final SparkMax feederMotor = new SparkMax(8, MotorType.kBrushless);

  public Command feed() {
    return this.runOnce(() -> feederMotor.set(-0.5));
  }

  public Command feed(double speed) {
    return this.runOnce(() -> feederMotor.set(speed));
  }

  public Command stop() {
    return this.runOnce(() -> feederMotor.set(0));
  }

  public double getSpeed() {
    return feederMotor.get();
  }
}
