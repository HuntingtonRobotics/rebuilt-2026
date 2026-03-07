package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Subsystem for the intake device.
 * 
<p>There are two actions concerning intake: deploying the intake device and spinning the intake rollers.
 * It is a style choice to make these actions part of the same subsystem class; the actions are part of the
 * same physical device, so two separate code methods (one per motor) is perhaps less overhead than two separate classes. </p>
    
*/
public class IntakeSubsystem extends SubsystemBase {
    
    private static final String IntakeDeployerTargetPosDashboardKey = "Intake Deployer Target Position";
    public static final String IntakeDeployerCurrentPosDashboardKey = "Intake Deployer Current Position";
    public static final String ResetEncoderDashboardKey = "Reset Intake Deployer Encoder";

    // Collector motor (rollers)
    private final SparkMax collectorMotor;

    // Deployer motor with closed-loop control
    private SparkMax deployMotor;
    private SparkMaxConfig motorConfig;
    private SparkClosedLoopController closedLoopController;
    private RelativeEncoder encoder;

    public IntakeSubsystem() {
        collectorMotor = new SparkMax(51, MotorType.kBrushless);
        deployMotor = new SparkMax(56, MotorType.kBrushless);
        closedLoopController = deployMotor.getClosedLoopController();
        encoder = deployMotor.getEncoder();
        motorConfig = new SparkMaxConfig();

        motorConfig.encoder
            .positionConversionFactor(1);
        motorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            // Set PID values for position control. We don't need to pass a closed loop
            // slot, as it will default to slot 0.
            .p(0.1)
            .i(0)
            .d(0)
            .outputRange(-1, 1);
        deployMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        SmartDashboard.setDefaultNumber(IntakeDeployerTargetPosDashboardKey, 0);
        SmartDashboard.setDefaultBoolean(ResetEncoderDashboardKey, false);
    }

    // ------------------- Collector (rollers) methods -------------------

    /**
     * Spin the collector motor at a given speed. Positive is intake, negative is outtake.
     *
     * @param speed The speed to spin the motor at, between -1 and 1.
     * @return A command that spins the motor at the given speed.
     */
    public Command spin(double speed) {
        return this.runOnce(() -> collectorMotor.set(speed));
    }

    /**
     * Stop the collector motor.
     *
     * @return A command that stops the collector.
     */
    public Command stop() {
        return this.runOnce(() -> collectorMotor.set(0));
    }

    /**
     * Run the collector at a fixed intake speed until ended.
     *
     * @return A command that starts the collector and stops it when finished.
     */
    public Command run() {
        return this.startEnd(
          () -> collectorMotor.set(0.5),
          () -> collectorMotor.set(0)
        );
    }

    // ------------------- Deploy (rotate in/out) methods -------------------
    // These two methods are the same but with different Dashboard defaults
    public Command deploy() {
        double targetPosition = SmartDashboard.getNumber(IntakeDeployerTargetPosDashboardKey, 0);
        return this.runOnce(() -> closedLoopController.setSetpoint(targetPosition, ControlType.kPosition));
    }

    public Command retract() {
        double targetPosition = SmartDashboard.getNumber(IntakeDeployerTargetPosDashboardKey, 0);
        return this.runOnce(() -> closedLoopController.setSetpoint(targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0));
    }
    
    public Command stopDeploy() {
        return this.runOnce(() -> deployMotor.set(0));
    }

    /**
     * Deploy or retract at a given speed. Automatically scaled down by 50%!
     */
    public Command runDeploy(double speed) {
        return this.startEnd(() -> deployMotor.set(speed * 0.5), () -> deployMotor.set(0));
    }

    public void resetEncoder() {
        encoder.setPosition(0);
    }

    public double getPosition() {
        return encoder.getPosition();
    }
}
