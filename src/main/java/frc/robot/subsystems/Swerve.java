package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;
import frc.robot.generated.Telemetry;
import frc.robot.generated.TunerConstants;


public class Swerve {
    private double MaxSpeed = Constants.SwerveConstants.MaxSpeed;
    private double MaxAngularRate = Constants.SwerveConstants.MaxAngularRate;

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public Swerve() {
    }

     public void configureBindings(CommandXboxController controller) {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-controller.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-controller.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-controller.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // When enabled, drive rotational motion based on vision (AprilTags)
        //  Use Left Joystick for translation (as usual)
        controller.rightBumper().whileTrue(
          Commands.run(() -> drivetrain.resetAngularPID())
          .andThen(
            drivetrain.applyRequest(() -> 
                drive.withVelocityX(-controller.getLeftY() * MaxSpeed)
                    .withVelocityY(-controller.getLeftX() * MaxSpeed)
                    .withRotationalRate(drivetrain.getAngularPID() * MaxAngularRate)
            )
          )
        );

        // Fine motor control
        fineMotorControlBindings(controller);

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        controller.a().whileTrue(drivetrain.applyRequest(() -> brake));
        controller.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-controller.getLeftY(), -controller.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        controller.back().and(controller.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        controller.back().and(controller.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        controller.start().and(controller.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        controller.start().and(controller.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        controller.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    private void fineMotorControlBindings(CommandXboxController controller) {
        double speedX = 0.25 * Constants.SwerveConstants.MaxSpeed;
        double speedY = 0.25 * Constants.SwerveConstants.MaxSpeed;

        // Forward
        controller.povUp().whileTrue(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(speedX)
            )
        );

        // Backward
        controller.povDown().whileTrue(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-speedX)
            )
        );

        // Left
        controller.povLeft().whileTrue(
            drivetrain.applyRequest(() ->
                drive.withVelocityY(speedY)
            )
        );

        // Right
        controller.povRight().whileTrue(
            drivetrain.applyRequest(() ->
                drive.withVelocityY(-speedY)
            )
        );

        // Forward-Left
        controller.povUpLeft().whileTrue(
            drivetrain.applyRequest(() -> 
                drive.withVelocityX(speedX)
                     .withVelocityY(speedY)    
            )
        );

        // Forward-Right
        controller.povUpRight().whileTrue(
            drivetrain.applyRequest(() -> 
                drive.withVelocityX(speedX)
                     .withVelocityY(-speedY)    
            )
        );

        // Backward-Left
        controller.povDownLeft().whileTrue(
            drivetrain.applyRequest(() -> 
                drive.withVelocityX(-speedX)
                     .withVelocityY(speedY)    
            )
        );

        // Backward-Right
        controller.povDownRight().whileTrue(
            drivetrain.applyRequest(() -> 
                drive.withVelocityX(-speedX)
                     .withVelocityY(-speedY)
            )
        );
    }
    
    public Command getSwerveAuto() {
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
            // Reset our field centric heading to match the robot
            // facing away from our alliance station wall (0 deg).
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            // Then slowly drive forward (away from us) for 5 seconds.
            drivetrain.applyRequest(() ->
                drive.withVelocityX(0.5)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            )
            .withTimeout(5.0),
            // Finally idle for the rest of auton
            drivetrain.applyRequest(() -> idle)
        );
    }
}
