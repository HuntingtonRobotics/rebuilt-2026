package frc.robot.subsystems.Shooter;
/*  
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FlywheelHood extends SubsystemBase {
    /* 
    public static final String ResetEncoderDashboardKey = "Reset Hood Position Encoder";
    private SparkMax motor;
    private SparkMaxConfig motorConfig;
    private SparkClosedLoopController closedLoopController;
    private RelativeEncoder encoder;

    public FlywheelHood() {
        motor = new SparkMax(53, MotorType.kBrushless);
        closedLoopController = motor.getClosedLoopController();
        encoder = motor.getEncoder();
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
            .outputRange(-10, 20);
        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    
        SmartDashboard.setDefaultBoolean(ResetEncoderDashboardKey, false);
    }

    // These two methods are the same but with different Dashboard defaults
    public Command high() {
        return this.runOnce(() -> closedLoopController.setSetpoint(9.8, ControlType.kPosition, ClosedLoopSlot.kSlot0));
    }

    public Command low() {
        return this.runOnce(() -> closedLoopController.setSetpoint(-0.1, ControlType.kPosition, ClosedLoopSlot.kSlot0));
    }

    public void resetEncoder() {
        encoder.setPosition(0);
    }

    public double getPosition() {
        return encoder.getPosition();
    }
   /* */ 