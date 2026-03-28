
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeCollector extends SubsystemBase {

  private final SparkMax leftcollectorMotor = new SparkMax(9, MotorType.kBrushless);
  private final SparkMax rightcollectorMotor = new SparkMax(8, MotorType.kBrushless);

  /**
   * Spin the motor at a given speed. Positive is intake, negative is outtake.
   * 
   * @param speed The speed to spin the motor at, between -1 and 1.
   * @return A command that spins the motor at the given speed.
   */
  public Command spin(double speed) {
    return this.runOnce(() -> {
      leftcollectorMotor.set(speed);
      rightcollectorMotor.set(speed);
    });
  }

  public Command stop() {
    return this.runOnce(() -> {
      leftcollectorMotor.set(0);
      rightcollectorMotor.set(0);
    });
  }

  public Command run() {
    return Commands.parallel(
      this.startEnd(() -> leftcollectorMotor.set(0.5), () -> leftcollectorMotor.set(0)), 
      this.startEnd(() -> rightcollectorMotor.set(0.5), () -> rightcollectorMotor.set(0))
    ); 
    
  }
}
