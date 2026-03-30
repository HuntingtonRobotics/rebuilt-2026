package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.WaitForSpeedCommand;

/** Subsystem for the flywheel shooter.
 * 
 * <p>
 * The shooter consists of two independent motors spinning separate flywheels.
 * </p>
 */
public class FlywheelShooter extends SubsystemBase {
    private static final double rightRpm = 2500;
    private static final double leftRpm = 2500;
    private static final double rightRotationsPerSecond = rightRpm/60;
    private static final double leftRotationsPerSecond = leftRpm/60;
    private static final double accelRPS = 30;
    private final TalonFX krakenMotorLeft = new TalonFX(61);
    private final TalonFX krakenMotorRight = new TalonFX(60);
    private final SparkMax acceleratorMotor = new SparkMax(59, MotorType.kBrushless); //change channel
    private final VelocityVoltage velocityVoltage = new VelocityVoltage(0).withSlot(0);
    private final NeutralOut brake = new NeutralOut();

    public FlywheelShooter() {
        // https://github.com/CrossTheRoadElec/Phoenix6-Examples/blob/main/java/VelocityClosedLoop/src/main/java/frc/robot/Robot.java
        TalonFXConfiguration config = new TalonFXConfiguration();
        /* Voltage-based velocity requires a velocity feed forward to account for the back-emf of the motor */
        config.Slot0.kS = 0.1; // To account for friction, add 0.1 V of static feedforward
        config.Slot0.kV = 0.12; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / rotation per second
        config.Slot0.kP = 0.11; // An error of 1 rotation per second results in 0.11 V output
        config.Slot0.kI = 0.0000025; // No output for integrated error
        config.Slot0.kD = 0.000025; // No output for error derivative
    
        // Peak output of 8 volts
        config.Voltage.withPeakForwardVoltage(50.0)
            .withPeakReverseVoltage(-50.0);

        /* Retry config apply up to 5 times, report if failure */
        applyConfig(krakenMotorLeft, config);
        applyConfig(krakenMotorRight, config);
    }

    public Command shoot() {
        return new ParallelCommandGroup(
            // setControl only needs to be called once to keep velocity
            this.runOnce(() -> {
                krakenMotorLeft.setControl(velocityVoltage.withVelocity(leftRotationsPerSecond));
                krakenMotorRight.setControl(velocityVoltage.withVelocity(-rightRotationsPerSecond));
                acceleratorMotor.set(accelRPS);
            }),
            new WaitForSpeedCommand(krakenMotorLeft, leftRotationsPerSecond, 0.1),
            new WaitForSpeedCommand(krakenMotorRight, rightRotationsPerSecond, 0.1)
        );
    }

    public Command shoot(double speed) {
        return this.runOnce(() -> {
            krakenMotorLeft.setControl(velocityVoltage.withVelocity(speed * 50));
            krakenMotorRight.setControl(velocityVoltage.withVelocity(speed * -50));
            acceleratorMotor.set(accelRPS);
        });
    }
    
    public Command stop() {
        return this.runOnce(() -> {
            krakenMotorLeft.setControl(brake);
            krakenMotorRight.setControl(brake);
            acceleratorMotor.stopMotor();
        });
    }

    public StatusSignal<AngularVelocity> getSpeedLeft() {
        return krakenMotorLeft.getVelocity();
    }

    public StatusSignal<AngularVelocity> getSpeedRight() {
        return krakenMotorRight.getVelocity();
    }

    private void applyConfig(TalonFX motor, TalonFXConfiguration config) {
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
          status = motor.getConfigurator().apply(config);
          if (status.isOK()) break;
        }
        if (!status.isOK()) {
          System.out.println("Could not apply configs, error code: " + status.toString());
        }
    }

}
//jake is the best ( ͡° ͜ʖ ͡°) ༼ ◥◣_◢◤ ༽ ✿∗˵╰༼✪ᗜ✪༽╯˵∗✿ Kieran is also here ᕙ(▀̿ĺ̯▀̿ ̿)ᕗ