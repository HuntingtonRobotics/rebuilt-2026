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
import frc.robot.Robot;

/** Subsystem for the intake device.
 * 
<p>There are two actions concerning intake: deploying the intake device and spinning the intake rollers.
 * It is a style choice to make these actions part of the same subsystem class; the actions are part of the
 * same physical device, so two separate code methods (one per motor) is perhaps less overhead than two separate classes. </p>
    
*/
public class IntakeSubsystemCollector extends SubsystemBase {
    
    private static final String IntakeDeployerTargetPosDashboardKey = "Intake Deployer Target Position";
    public static final String IntakeDeployerCurrentPosDashboardKey = "Intake Deployer Current Position";
    public static final String ResetEncoderDashboardKey = "Reset Intake Deployer Encoder";

    // Collector motor (rollers)
    private SparkMax collectorMotor;
    // Deployer motor with closed-loop control
    //  there are two physical motors on robot but only one needs to be configured here; the other is set to follow

    public IntakeSubsystemCollector() {
        if (!Robot.isSimulation()) {}
        collectorMotor = new SparkMax(52, MotorType.kBrushless);
        
        
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
    public Command spin() {
      return this.runOnce(() -> collectorMotor.set(-0.8));
    }

    public Command reverseSpin() {
      return this.runOnce(() -> collectorMotor.set(0.1));
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
      return this.runOnce(
        () -> collectorMotor.set(-1)
      );
    }


    // ------------------- Deploy (rotate in/out) methods -------------------
    // These two methods are the same but with different Dashboard defaults
   // public Command flexIntake() {
     //   if(encoder.getPosition() > 6){
       //     return Commands.sequence(retract(),new WaitCommand(7),
 //deploy());
   //     }
     //   return Commands.sequence(deploy(),new WaitCommand(7),
 //retract());
  //  }

    /**
     * Deploy or retract at a given speed
     */
}