package frc.robot.subsystems;
import frc.robot.subsystems.LimelightHelpers;
public class LimelightCamera {
    // *** NOTE: In Limelight world, +Y is forward, +X is right ***
    //  this is different than FRC in which +X is forward and +Y is left
    private final double MaxSpeed;
    private final double MaxAngularRate;

    public LimelightCamera(double maxSpeed, double maxAngularRate) {
        MaxSpeed = maxSpeed;
        MaxAngularRate = maxAngularRate;
    }

    // Calculate proportional aim (rotation) speed
    public double aimProportional() {
        // Constant of proportionality
        double kP = 0.035;
        setPipeline();

        double targetingAngularVelocity = LimelightHelpers.getTX("limelight") * kP;
        targetingAngularVelocity *= MaxAngularRate / 2;
        targetingAngularVelocity *= -1.0; // -1.0 is for controller invert
        return targetingAngularVelocity;
    }

    // Calculate proportional range (drive) speed 
    public double rangeProportional() {
        // Constant of proportionality; control effective distance from tag
        double kP = 0.1;
        setPipeline();

        double targetingForwardSpeed = LimelightHelpers.getTY("limelight") * kP;
        targetingForwardSpeed *= (MaxSpeed / 6);
        targetingForwardSpeed *= -1.0;
        return targetingForwardSpeed;
    }

    private void setPipeline() {
        int pipelineIndex = 0; // See Limelight config page
        LimelightHelpers.setPipelineIndex("limelight", pipelineIndex);
    }
}
