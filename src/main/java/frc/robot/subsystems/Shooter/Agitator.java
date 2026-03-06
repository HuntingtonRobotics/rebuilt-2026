package frc.robot.subsystems.Shooter;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Agitator extends SubsystemBase {

  // *** CHANGE IDs ***
  //  set motorLeft to FOLLOW in REV UI
  // private final SparkMax motorLeft = new SparkMax(6, MotorType.kBrushless);
  private final SparkMax motorRight = new SparkMax(7, MotorType.kBrushless);

  public Command shakeIt() {
    return this.startEnd(
      () -> {
        motorRight.set(0.5);
      },
      () -> {
        motorRight.set(0);
      }
    );
  }

  public Command stop() {
    return this.runOnce(() -> motorRight.set(0));
  }
}
