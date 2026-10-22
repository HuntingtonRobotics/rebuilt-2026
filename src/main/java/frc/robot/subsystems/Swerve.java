package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;
import frc.robot.generated.Telemetry;
import frc.robot.generated.TunerConstants;
import frc.robot.util.LimelightAimer;

public class Swerve extends SubsystemBase{
    private double MaxSpeed = Constants.SwerveConstants.MaxSpeed;
    private double MaxAngularRate = Constants.SwerveConstants.MaxAngularRate;

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final LimelightAimer aimer = new LimelightAimer();
    // -------------------------------------------------------
    // Limelight Aim
    // -------------------------------------------------------

    private void setPipeline() {
        LimelightHelpers.setPipelineIndex("limelight", 0);
    }

    /**
     * Returns a proportional rotational rate to center the robot on the limelight target.
     * Positive TX = target is to the right = rotate right (negative rate).
     */
    public double aimProportional() {
        double kP = 0.035;
        setPipeline();
        double targetingAngularVelocity = LimelightHelpers.getTX("limelight") * kP;
        //println("Limelight TX", tx);
        targetingAngularVelocity *= MaxAngularRate / 2;
        targetingAngularVelocity *= -1.0;
        return targetingAngularVelocity;
    }

    public Swerve() {
    }



    public void applyVisionDrive(double velocityX, double velocityY, double rotRate) {
    drivetrain.setControl(
        drive.withVelocityX(velocityX)
             .withVelocityY(velocityY)
             .withRotationalRate(rotRate)
        );
    }   

    public void configureBindings(CommandXboxController controller, CommandXboxController operatorController) {
        // Default drive command — full manual control
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-controller.getLeftY() * MaxSpeed)
                    .withVelocityY(-controller.getLeftX() * MaxSpeed)
                    .withRotationalRate(
                        operatorController.rightBumper().getAsBoolean()
                            ? aimer.getRotation(controller.getRightX()) * MaxAngularRate
                            : -controller.getRightX() * MaxAngularRate
                    )
            )
        );


        // Right bumper: vision-assisted rotation using angular PID
        // controller.rightBumper().whileTrue(
        //     Commands.run(() -> drivetrain.resetAngularPID())
        //     .andThen(
        //         drivetrain.applyRequest(() ->
        //             drive.withVelocityX(-controller.getLeftY() * MaxSpeed)
        //                 .withVelocityY(-controller.getLeftX() * MaxSpeed)
        //                 .withRotationalRate(drivetrain.getAngularPID() * MaxAngularRate)
        //         )
        //     )
        // );

        // Y button: limelight tag tracking
        // Left stick still controls forward/backward and strafe
        // Rotation is fully taken over by limelight proportional aim
        controller.y().whileTrue(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-controller.getLeftY() * MaxSpeed)
                    .withVelocityY(-controller.getLeftX() * MaxSpeed)
                    .withRotationalRate(aimProportional())
            )
        );

        // Left bumper: reset field-centric heading
        controller.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        fineMotorControlBindings(controller);

        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        controller.a().whileTrue(drivetrain.applyRequest(() -> brake));

        controller.back().and(controller.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        controller.back().and(controller.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        controller.start().and(controller.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        controller.start().and(controller.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    private void fineMotorControlBindings(CommandXboxController controller) {
        double speedX = 0.25 * Constants.SwerveConstants.MaxSpeed;
        double speedY = 0.25 * Constants.SwerveConstants.MaxSpeed;

        controller.povUp().whileTrue(
            drivetrain.applyRequest(() -> drive.withVelocityX(speedX))
        );
        controller.povDown().whileTrue(
            drivetrain.applyRequest(() -> drive.withVelocityX(-speedX))
        );
        controller.povLeft().whileTrue(
            drivetrain.applyRequest(() -> drive.withVelocityY(speedY))
        );
        controller.povRight().whileTrue(
            drivetrain.applyRequest(() -> drive.withVelocityY(-speedY))
        );
        controller.povUpLeft().whileTrue(
            drivetrain.applyRequest(() -> drive.withVelocityX(speedX).withVelocityY(speedY))
        );
        controller.povUpRight().whileTrue(
            drivetrain.applyRequest(() -> drive.withVelocityX(speedX).withVelocityY(-speedY))
        );
        controller.povDownLeft().whileTrue(
            drivetrain.applyRequest(() -> drive.withVelocityX(-speedX).withVelocityY(speedY))
        );
        controller.povDownRight().whileTrue(
            drivetrain.applyRequest(() -> drive.withVelocityX(-speedX).withVelocityY(-speedY))
        );
    }

    // -------------------------------------------------------
    // Movement Commands
    // -------------------------------------------------------

    public Command moveForwardWithLateral(double forwardSpeed, double lateralSpeed, double durationSecs) {
        return drivetrain.applyRequest(() ->
            drive.withVelocityX(forwardSpeed)
                 .withVelocityY(lateralSpeed)
                 .withRotationalRate(0)
        ).withTimeout(durationSecs);
    }

    public Command moveForward(double speed, double durationSecs) {
        return moveForwardWithLateral(speed, 0.0, durationSecs);
    }

    public Command moveForwardLeft(double speed, double durationSecs) {
        return moveForwardWithLateral(speed, speed, durationSecs);
    }

    public Command moveForwardRight(double speed, double durationSecs) {
        return moveForwardWithLateral(speed, -speed, durationSecs);
    }

    // -------------------------------------------------------
    // Turn Commands
    // -------------------------------------------------------

    public Command turn(double turnRate, double durationSecs) {
        return drivetrain.applyRequest(() ->
            drive.withVelocityX(0)
                 .withVelocityY(0)
                 .withRotationalRate(turnRate)
        ).withTimeout(durationSecs);
    }

    public Command turnLeft(double turnRate, double durationSecs) {
        return turn(Math.abs(turnRate), durationSecs);
    }

    public Command turnRight(double turnRate, double durationSecs) {
        return turn(-Math.abs(turnRate), durationSecs);
    }

    public Command turnToTagCommand(java.util.function.DoubleSupplier rotationSupplier) {
        return drivetrain.applyRequest(() ->
            drive.withVelocityX(0)
                 .withVelocityY(0)
                 .withRotationalRate(rotationSupplier.getAsDouble())
        );
    }
    
    // -------------------------------------------------------
    // Auto & Telemetry
    // -------------------------------------------------------

    public Command getSwerveAuto() {
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            drivetrain.applyRequest(() ->
                drive.withVelocityX(0.5)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            )
            .withTimeout(5.0),
            drivetrain.applyRequest(() -> idle)
        );
    }

    public Command homeIn(){
        return drivetrain.runOnce(() ->
            drive.withRotationalRate(aimer.getRotation(0) * MaxAngularRate)
        );
    }
}