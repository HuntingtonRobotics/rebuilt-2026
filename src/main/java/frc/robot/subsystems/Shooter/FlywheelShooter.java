package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.pid;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.WaitForSpeedCommand;
import frc.robot.util.LimelightAimer;

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
    private final TalonFX krakenMotorTop = new TalonFX(61);
    private final TalonFX krakenMotorBottom = new TalonFX(60);
    private final VelocityVoltage velocityVoltage = new VelocityVoltage(0).withSlot(0);
    private final NeutralOut brake = new NeutralOut();
    private final LimelightAimer aimer = new LimelightAimer();
    private double distance = aimer.getDistance(0);

    public FlywheelShooter() {

        //Benmakebadautos
        // https://github.com/CrossTheRoadElec/Phoenix6-Examples/blob/main/java/VelocityClosedLoop/src/main/java/frc/robot/Robot.java
        TalonFXConfiguration config = new TalonFXConfiguration();
        /* Voltage-based velocity requires a velocity feed forward to account for the back-emf of the motor */
        config.Slot0.kS = 0.1; // To account for friction, add 0.1 V of static feedforward
        config.Slot0.kV = 0.12; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / rotation per second
        config.Slot0.kP = 0.1; 
        config.Slot0.kI = 0.0; // No output for integrated error
        config.Slot0.kD = 0.0125; 
    
        // Peak output of 8 volts
        config.Voltage.withPeakForwardVoltage(12.0)
            .withPeakReverseVoltage(-12.0);

        /* Retry config apply up to 5 times, report if failure */
        applyConfig(krakenMotorTop, config);
        applyConfig(krakenMotorBottom, config);

    }


    public Command shoot() {
        return this.runOnce(() -> {
            krakenMotorTop.setControl(velocityVoltage.withVelocity(leftRotationsPerSecond));
            krakenMotorBottom.setControl(velocityVoltage.withVelocity(-rightRotationsPerSecond));
        });
    }

    public Command shoot(double speed) {
        return this.runOnce(() -> {
            krakenMotorTop.setControl(velocityVoltage.withVelocity(speed/60)); //black wheels
            krakenMotorBottom.setControl(velocityVoltage.withVelocity(speed / -60)); //blue wheels
        });
    }

    public Command shootWithPID() {
        
        return this.run(() -> {

            distance = aimer.getDistance(distance);
            double rpmTop = rpmFromDistance(distance);
            double rpmBottom = rpmFromDistance(distance);
            //System.out.println(krakenMotorBottom.getVelocity().getValueAsDouble() * 60.0 - rpmFromDistance(aimer.getDistance(0)));
            System.out.println("Distance: " + distance);
            krakenMotorTop.setControl(velocityVoltage.withVelocity(
                (-rpmTop / 60.0) 
            ));
            krakenMotorBottom.setControl(velocityVoltage.withVelocity(
                (rpmBottom / 60.0)
            ));
        });
    }

    public Command stop() {
        return this.runOnce(() -> {
            krakenMotorBottom.set(0);
            krakenMotorTop.set(0);
            krakenMotorTop.setControl(brake);
            krakenMotorBottom.setControl(brake);
        });
    }

    public void setSpeed(double speed) {
        krakenMotorTop.setControl(velocityVoltage.withVelocity(speed * 50));
        krakenMotorBottom.setControl(velocityVoltage.withVelocity(speed * -50));
    }

    public StatusSignal<AngularVelocity> getSpeedLeft() {
        return krakenMotorTop.getVelocity();
    }

    public StatusSignal<AngularVelocity> getSpeedRight() {
        return krakenMotorBottom.getVelocity();
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

    public double rpmFromDistance(double distance) {
        double a = 2.29961;
        double b = 52.26073;
        double c = 2875.09104;
        return a * Math.pow(distance, 2) + b * distance + c; // equation that ben and dylan came up with to convert distance to rpm
    }
    public double getRPMError() {
        return Math.abs(krakenMotorBottom.getVelocity().getValueAsDouble() * 60.0 - rpmFromDistance(aimer.getDistance(0))); // convert from rotations per second to rpm
    }
}
