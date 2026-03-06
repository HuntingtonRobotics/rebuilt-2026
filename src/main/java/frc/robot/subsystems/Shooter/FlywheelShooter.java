package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.WaitForSpeedCommand;

public class FlywheelShooter extends SubsystemBase {

    private static final double RotationsPerSecond = 41.7;
    private final TalonFX krakenMotor = new TalonFX(1);
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
        config.Voltage.withPeakForwardVoltage(8.0)
            .withPeakReverseVoltage(-8.0);

        /* Retry config apply up to 5 times, report if failure */
        applyConfig(config);
    }

    public Command shoot() {
        return new ParallelCommandGroup(
            // setControl only needs to be called once to keep velocity
            this.runOnce(() -> krakenMotor.setControl(velocityVoltage.withVelocity(RotationsPerSecond))),
            new WaitForSpeedCommand(krakenMotor, RotationsPerSecond, 0.1)
        );
    }

    public Command shoot(double speed) {
        return this.runOnce(() -> krakenMotor.set(speed));
    }
    
    public Command stop() {
        return this.runOnce(() -> krakenMotor.setControl(brake));
    }

    private void applyConfig(TalonFXConfiguration config) {
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
          status = krakenMotor.getConfigurator().apply(config);
          if (status.isOK()) break;
        }
        if (!status.isOK()) {
          System.out.println("Could not apply configs, error code: " + status.toString());
        }
    }

}
