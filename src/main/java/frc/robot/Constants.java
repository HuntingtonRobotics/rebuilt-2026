// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.generated.TunerConstants;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import static edu.wpi.first.units.Units.*;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    // Port numbers for driver and operator gamepads. These correspond with the numbers on the USB
    // tab of the DriverStation
    public static final int DriverControllerPort = 0;
    public static final int OperatorControllerPort = 1;
  }
  public static class VisionConstants {
    // Limelight name (match what's configured at http://limelight.local)
    public static final String LIMELIGHT_NAME = "";

    // AprilTag IDs to target
    public static final int[] TARGET_TAG_IDS = {12, 15};

    // Rotation PID - controls left/right alignment to center tag (tx = 0)
    public static final double APRILTAG_ROTATION_KP = 0.02;
    public static final double APRILTAG_ROTATION_KI = 0.0;
    public static final double APRILTAG_ROTATION_KD = 0.001;
    public static final double APRILTAG_ROTATION_DEADBAND = 2.0; // degrees

    // Drive PID - controls distance using target area
    public static final double APRILTAG_DRIVE_KP = 0.05;
    public static final double APRILTAG_DRIVE_KI = 0.0;
    public static final double APRILTAG_DRIVE_KD = 0.001;
    public static final double APRILTAG_TARGET_AREA = 2.0;   // % of frame, tune this
    public static final double APRILTAG_AREA_TOLERANCE = 0.3; // % tolerance

    // Speed limits - these are FRACTIONS of MaxSpeed/MaxAngularRate (0.0 to 1.0)
    // since CTRE scales by MaxSpeed and MaxAngularRate
    public static final double APRILTAG_MAX_ROTATION_SPEED = 0.4;
    public static final double APRILTAG_MIN_ROTATION_SPEED = 0.05;
    public static final double APRILTAG_MAX_DRIVE_SPEED = 0.3;
    public static final double APRILTAG_MIN_DRIVE_SPEED = 0.05;
}
  public static class DashboardConstants {
    public static final String AutoModeKey = "Auto Mode";
    public static final String VisionOdoEnabledKey = "visionOdoEnabled";
  }

  public static class SwerveConstants {
    public static double MaxSpeed = 0.40 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    public static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
  }

  public static class FieldConstants {
    private static AprilTagFieldLayout layout;
    static {
      layout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    }

    private static Translation2d getTagPoseTranslation(int ID) {
      return layout.getTagPose(ID).get().toPose2d().getTranslation();
    }

    public static Translation2d HubRed = getTagPoseTranslation(10);
    public static Translation2d HubBlue = getTagPoseTranslation(25);

    /**
     * Returns the length (X-axis) of the field in meters.
     * -- From Team 340
     */
    public static double length() {
        return layout.getFieldLength();
    }

    /**
     * Returns the width (Y-axis) of the field in meters.
     * -- From Team 340
     */
    public static double width() {
        return layout.getFieldWidth();
    }

  }
}
