package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.VisionConstants;

public class LimelightAimer {
    private final PIDController m_rotationPID;
    private final int[] m_targetTags = {10, 26};
    private final double MaxAngularRate = Constants.SwerveConstants.MaxAngularRate;
    private double distance = getDistance(0);


    public LimelightAimer() {

        m_rotationPID = new PIDController(
            Constants.VisionConstants.APRILTAG_ROTATION_KP,
            Constants.VisionConstants.APRILTAG_ROTATION_KI,
            Constants.VisionConstants.APRILTAG_ROTATION_KD
        );
        m_rotationPID.setSetpoint(0);
        m_rotationPID.setTolerance(Constants.VisionConstants.APRILTAG_ROTATION_DEADBAND);
    }

    public void reset() {
        m_rotationPID.reset();
    }

    /**
     * Returns the rotation rate to aim at a valid AprilTag,
     * or falls back to the joystick value if no valid tag is seen.
     */
    public double getRotation(double joystickRightX) {

        if (!LimelightHelpers.getTV("limelight")) {
            return -joystickRightX * MaxAngularRate;
        }


        int tagID = (int) LimelightHelpers.getFiducialID("limelight");
        boolean isValidTag = false;
        for (int id : m_targetTags) {
            if (id == tagID) { isValidTag = true; break; }
        }

        if (!isValidTag) {
            return -joystickRightX * MaxAngularRate;
        }
        double tx = LimelightHelpers.getTX("limelight"); // override tx with angle to virtual target
        double deadband = 3.0; // degrees
        double maxRotation = 0.15;
        double minRotation = 0.0;

        if (Math.abs(tx) < deadband) return 0;

        double rotation = -tx * 0.1; // negate: positive tx → rotate right (negative)
        rotation = MathUtil.clamp(rotation, -0.2, 0.2);

        if (Math.abs(rotation) < minRotation) {
            rotation = Math.copySign(minRotation, rotation);
        }
        if (Math.abs(rotation) > maxRotation) {
            rotation = Math.copySign(maxRotation, rotation);
        }
        if (Math.abs(tx) < deadband * 3) {
            double scale = Math.abs(tx) / (deadband * 3);
            rotation *= Math.max(0.3, scale);
        }

        return rotation;
    }
    public double getDistance(double distance){
        if (!LimelightHelpers.getTV("limelight")) return distance; // no target
        return ((27.25/12.0) / Math.tan(Math.toRadians(18.0 + LimelightHelpers.getTY("limelight")))); // 1.14829 needs to be fixed distance in feet, 27.25 inches is the height difference between the limelight and the target, 18 degrees is the mounting angle of the limelight, 1.14829 is a correction factor that ben and dylan came up with to make the distance more accurate
    }

    public boolean ValidTarget() {
        return LimelightHelpers.getTV("limelight");
    }
}