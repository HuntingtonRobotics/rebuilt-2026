package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Swerve;
//test

public class April extends Command {
  private final Swerve m_driveSubsystem;
  private final PIDController m_rotationPID;
  private final PIDController m_drivePID;
  private final int[] m_targetTags = {2,5,10,18,21,26};
  private int executionCount = 0;
  public double MaxSpeed = Constants.SwerveConstants.MaxSpeed;
  public double MaxAngularRate = Constants.SwerveConstants.MaxAngularRate;
  // Try different possible Limelight names in NetworkTables
  private static final String[] POSSIBLE_LIMELIGHT_NAMES = {"limelight"};
  private String activeLimelightName = "";

  public April(Swerve driveSubsystem) {
    m_driveSubsystem = driveSubsystem;
    addRequirements(m_driveSubsystem);

    // Initialize PID controllers with constants
    m_rotationPID = new PIDController(
      frc.robot.Constants.VisionConstants.APRILTAG_ROTATION_KP,
      frc.robot.Constants.VisionConstants.APRILTAG_ROTATION_KI,
      frc.robot.Constants.VisionConstants.APRILTAG_ROTATION_KD
    );
    m_rotationPID.setSetpoint(0); // Target is centered when tx = 0
    m_rotationPID.setTolerance(frc.robot.Constants.VisionConstants.APRILTAG_ROTATION_DEADBAND);

    m_drivePID = new PIDController(
      frc.robot.Constants.VisionConstants.APRILTAG_DRIVE_KP,
      frc.robot.Constants.VisionConstants.APRILTAG_DRIVE_KI,
      frc.robot.Constants.VisionConstants.APRILTAG_DRIVE_KD
    );
    m_drivePID.setSetpoint(frc.robot.Constants.VisionConstants.APRILTAG_TARGET_AREA);
    m_drivePID.setTolerance(frc.robot.Constants.VisionConstants.APRILTAG_AREA_TOLERANCE);

    System.out.println("[AlignToAprilTag] Command created, targeting tags: 12, 15");
    System.out.println("[AlignToAprilTag] Target area: " + frc.robot.Constants.VisionConstants.APRILTAG_TARGET_AREA + "%, tolerance: " + frc.robot.Constants.VisionConstants.APRILTAG_AREA_TOLERANCE + "%");
    System.out.println("[AlignToAprilTag] Limelight IP: 10.80.92.200");
  }

  @Override
  public void initialize() {
    executionCount = 0;

    // Reset PID controllers to clear any accumulated error
    m_rotationPID.reset();
    m_drivePID.reset();

    System.out.println("[AlignToAprilTag] Command started - X button pressed");

    // Use default limelight (already verified to work)
    if (activeLimelightName.isEmpty()) {
      activeLimelightName = "";
    }

    // Force Limelight LEDs on for AprilTag detection
    LimelightHelpers.setLEDMode_ForceOn(activeLimelightName);

    // Quick status check
    boolean hasTarget = LimelightHelpers.getTV(activeLimelightName);
    System.out.println("[AlignToAprilTag] Initial target detected: " + hasTarget);
  }

  @Override
  public void execute() {
    executionCount++;
    boolean hasTarget = LimelightHelpers.getTV(activeLimelightName);
    double rotationSpeed = 0;
    double xSpeed = 0; // Forward/backward movement (X axis controls forward on this robot)

    // Debug: Print every 10 executions (~0.2 seconds at 50Hz)
    boolean shouldLog = (executionCount % 10 == 0);

    if (shouldLog) {
      System.out.println("[AlignToAprilTag] ========== Execution #" + executionCount + " ==========");
      System.out.println("[AlignToAprilTag] Has target: " + hasTarget);
    }

    if (hasTarget) {
      int currentTagID = (int) LimelightHelpers.getFiducialID(activeLimelightName);
      double tx = LimelightHelpers.getTX(activeLimelightName);
      double ty = LimelightHelpers.getTY(activeLimelightName);
      double ta = LimelightHelpers.getTA(activeLimelightName);

      if (shouldLog) {
        System.out.println("[AlignToAprilTag] Current Tag ID: " + currentTagID);
        System.out.println("[AlignToAprilTag] tx (horizontal offset): " + String.format("%.2f", tx) + " degrees");
        System.out.println("[AlignToAprilTag] ty (vertical offset): " + String.format("%.2f", ty) + " degrees");
        System.out.println("[AlignToAprilTag] ta (area): " + String.format("%.2f", ta) + "%");
      }

      boolean isValidTag = false;
      for (int id : m_targetTags) {
        if (id == currentTagID) {
          isValidTag = true;
          break;
        }
      }

      if (shouldLog) {
        System.out.println("[AlignToAprilTag] Is valid tag: " + isValidTag);
      }

      if (isValidTag) {
        // Get rotation constants
        double maxRotation = frc.robot.Constants.VisionConstants.APRILTAG_MAX_ROTATION_SPEED;
        double minRotation = frc.robot.Constants.VisionConstants.APRILTAG_MIN_ROTATION_SPEED;
        double deadband = frc.robot.Constants.VisionConstants.APRILTAG_ROTATION_DEADBAND;

        // Deadband check - don't rotate if already well-aligned
        if (Math.abs(tx) < deadband) {
          rotationSpeed = 0;
          if (shouldLog) {
            System.out.println("[AlignToAprilTag] Within deadband (" + String.format("%.2f", tx) + "° < " + deadband + "°), no rotation");
          }
        } else {
          // Calculate rotation speed to center the target (tx = 0)
          // Positive tx = target to right, need to rotate right (positive)
          rotationSpeed = m_rotationPID.calculate(tx);

          // Apply minimum speed only if error is significant
          if (Math.abs(rotationSpeed) < minRotation && Math.abs(tx) > deadband * 1.5) {
            rotationSpeed = Math.copySign(minRotation, rotationSpeed);
            if (shouldLog) {
              System.out.println("[AlignToAprilTag] Applied minimum rotation: " + String.format("%.3f", rotationSpeed));
            }
          }

          // Clamp to maximum rotation speed
          if (Math.abs(rotationSpeed) > maxRotation) {
            rotationSpeed = Math.copySign(maxRotation, rotationSpeed);
            if (shouldLog) {
              System.out.println("[AlignToAprilTag] Clamped to max rotation: " + String.format("%.3f", rotationSpeed));
            }
          }

          // Smooth scaling for small errors (proportional reduction)
          if (Math.abs(tx) < deadband * 3) {
            double scaleFactor = Math.abs(tx) / (deadband * 3);
            rotationSpeed *= Math.max(0.3, scaleFactor); // Never reduce below 30%
            if (shouldLog) {
              System.out.println("[AlignToAprilTag] Smooth scaling applied, factor: " + String.format("%.2f", scaleFactor));
            }
          }
        }

        // Calculate forward speed based on target area (distance)
        double maxDriveSpeed = frc.robot.Constants.VisionConstants.APRILTAG_MAX_DRIVE_SPEED;
        double minDriveSpeed = frc.robot.Constants.VisionConstants.APRILTAG_MIN_DRIVE_SPEED;
        double targetArea = frc.robot.Constants.VisionConstants.APRILTAG_TARGET_AREA;
        double areaTolerance = frc.robot.Constants.VisionConstants.APRILTAG_AREA_TOLERANCE;

        // PID calculates: error = setpoint - measurement = targetArea - ta
        // When ta is small (far away), error is positive
        double pidOutput = m_drivePID.calculate(ta);

        // Determine direction: if ta < target, we need to move FORWARD (positive)
        double distanceError = targetArea - ta;
        double forwardSpeed = 0;

      

        // At setpoint check
        if (m_drivePID.atSetpoint()) {
          forwardSpeed = 0;
          if (shouldLog) System.out.println("[AlignToAprilTag] At setpoint, stopping");
          //calculate shooting here
        } else if (distanceError > areaTolerance) {
          // Too far, need to move forward (NEGATIVE speed for this robot's X axis)
          forwardSpeed = -Math.abs(pidOutput);

          // Apply minimum speed
          if (Math.abs(forwardSpeed) < minDriveSpeed) {
            forwardSpeed = -minDriveSpeed;
            if (shouldLog) System.out.println("[AlignToAprilTag] Applied minimum forward: " + String.format("%.3f", forwardSpeed));
          }

          // Clamp to max
          if (Math.abs(forwardSpeed) > maxDriveSpeed) {
            forwardSpeed = -maxDriveSpeed;
            if (shouldLog) System.out.println("[AlignToAprilTag] Clamped to max forward: " + String.format("%.3f", forwardSpeed));
          }

          if (shouldLog) System.out.println("[AlignToAprilTag] Moving FORWARD (ta < target, X axis negative)");

        } else if (distanceError < -areaTolerance) {
          // Too close, need to move backward (POSITIVE speed for this robot's X axis)
          forwardSpeed = Math.abs(pidOutput);

          // Apply minimum speed (backward)
          if (Math.abs(forwardSpeed) < minDriveSpeed) {
            forwardSpeed = minDriveSpeed;
            if (shouldLog) System.out.println("[AlignToAprilTag] Applied minimum backward: " + String.format("%.3f", forwardSpeed));
          }

          // Clamp to max
          if (Math.abs(forwardSpeed) > maxDriveSpeed) {
            forwardSpeed = maxDriveSpeed;
            if (shouldLog) System.out.println("[AlignToAprilTag] Clamped to max backward: " + String.format("%.3f", forwardSpeed));
          }

          if (shouldLog) System.out.println("[AlignToAprilTag] Moving BACKWARD (ta > target, X axis positive)");
        }

        // Allow forward movement while aligning - only slow down if very misaligned
        double originalForward = forwardSpeed;
        if (Math.abs(tx) > 15.0) {
          forwardSpeed *= 0.5; // Slow down forward movement when significantly misaligned
        } else if (Math.abs(tx) > 8.0) {
          forwardSpeed *= 0.7; // Moderate slowdown when moderately misaligned
        }

        if (shouldLog && forwardSpeed != originalForward) {
          System.out.println("[AlignToAprilTag] Reduced for alignment (tx=" + String.format("%.2f", tx) + "°): " + String.format("%.3f", forwardSpeed));
        }

        // Use forward speed as X-axis (this robot uses X axis for forward/backward)
        xSpeed = forwardSpeed;

        if (shouldLog) {
          System.out.println("[AlignToAprilTag] Rotation PID: " + String.format("%.3f", rotationSpeed) + " (tx: " + String.format("%.2f", tx) + "°)");
          System.out.println("[AlignToAprilTag] Final forward speed (X axis): " + String.format("%.3f", xSpeed));
          System.out.println("[AlignToAprilTag] Rotation aligned: " + m_rotationPID.atSetpoint());
          System.out.println("[AlignToAprilTag] Distance aligned: " + m_drivePID.atSetpoint());
          if (m_rotationPID.atSetpoint() && m_drivePID.atSetpoint()) {
            System.out.println("[AlignToAprilTag] *** AT TARGET POSITION ***");
          }
        }
      } else {
        if (shouldLog) {
          System.out.println("[AlignToAprilTag] Detected tag is not 12 or 15, not aligning");
        }
      }

      // Update SmartDashboard for Driver Station
      SmartDashboard.putNumber("Vision/TagID", currentTagID);
      SmartDashboard.putNumber("Vision/TX", tx);
      SmartDashboard.putNumber("Vision/TA", ta);
      SmartDashboard.putBoolean("Vision/ValidTag", isValidTag);
      SmartDashboard.putNumber("Vision/RotationSpeed", rotationSpeed);
      SmartDashboard.putNumber("Vision/ForwardSpeed", xSpeed);
      SmartDashboard.putBoolean("Vision/AtTarget", m_rotationPID.atSetpoint() && m_drivePID.atSetpoint());
    } else {
      if (shouldLog) {
        System.out.println("[AlignToAprilTag] No AprilTag detected by Limelight");
      }
      SmartDashboard.putNumber("Vision/TagID", -1);
      SmartDashboard.putBoolean("Vision/ValidTag", false);
    }

    // Use the raw drive method to bypass deadzone and scaling
    // driveRaw(xSpeed, ySpeed, rotation)
    // For this robot: X axis controls forward/backward, Y axis controls strafe
    m_driveSubsystem.applyVisionDrive(xSpeed * MaxSpeed, 0, rotationSpeed * MaxAngularRate);



    if (shouldLog) {
      System.out.println("[AlignToAprilTag] Commands -> X (Forward): " + String.format("%.3f", xSpeed) + ", Rotation: " + String.format("%.3f", rotationSpeed));
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.applyVisionDrive(0, 0, 0);


    System.out.println("[AlignToAprilTag] Command ended - X button released (interrupted: " + interrupted + ")");
    System.out.println("[AlignToAprilTag] Total executions: " + executionCount);
  }

  @Override
  public boolean isFinished() {
    return false; // Run until button is released
  }
}
