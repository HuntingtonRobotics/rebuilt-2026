// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ShootUntilEmpty;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Intake.IntakeCollector;
import frc.robot.subsystems.Intake.IntakeDeploy;
import frc.robot.subsystems.Shooter.Agitator;
import frc.robot.subsystems.Shooter.Feeder;
import frc.robot.subsystems.Shooter.FlywheelHood;
import frc.robot.subsystems.Shooter.FlywheelShooter;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // Subsystems
  private final Swerve swerveDrivetrain = new Swerve();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final IntakeDeploy intakeDeploy = new IntakeDeploy();
  private final IntakeCollector intakeCollector = new IntakeCollector();
  private final Agitator agitator = new Agitator();
  private final Feeder shooterFeeder = new Feeder();
  private final FlywheelShooter shooter = new FlywheelShooter();
  private final FlywheelHood shooterHood = new FlywheelHood();

  // Commands
  private final ShootUntilEmpty shootUntilEmpty = new ShootUntilEmpty(agitator, shooterFeeder, shooter);

  private final CommandXboxController driverController =
    new CommandXboxController(OperatorConstants.DriverControllerPort);
  private final CommandXboxController operatorController =
    new CommandXboxController(OperatorConstants.OperatorControllerPort);

  /* Path follower */
  private final SendableChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    registerDashboardProperties();
    registerNamedCommands();
    autoChooser = AutoBuilder.buildAutoChooser("shootUntilEmpty");

    configureBindings();

    // Add camera feed to dashboard
    CameraServer.startAutomaticCapture();
  }

  private void registerNamedCommands() {
    NamedCommands.registerCommand("shootUntilEmpty", shootUntilEmpty);
  }

  private void registerDashboardProperties() {
    SmartDashboard.putBoolean(Constants.DashboardConstants.VisionOdoEnabledKey, true);
  }

  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition).onTrue(
    //   new ExampleCommand(m_exampleSubsystem)
    // );

    configureDriveBindings();
    configureGameplayBindings();
  }

  private void configureDriveBindings() {
    swerveDrivetrain.configureBindings(driverController);
  }

  private void configureGameplayBindings() {
    //TODO - refactor into methods
    /* Intake */
    //  Deploy
    operatorController.povUp()
      .onTrue(intakeSubsystem.deploy())
      .onFalse(intakeSubsystem.stopDeploy());
    operatorController.povDown()
      .onTrue(intakeSubsystem.retract())
      .onFalse(intakeSubsystem.stopDeploy());
    //  Deploy (fine control, fixed speed)
    operatorController.povUp().and(operatorController.a()
      .onTrue(intakeSubsystem.runDeploy(0.25)))
      .onFalse(intakeSubsystem.stopDeploy());
    operatorController.povDown().and(operatorController.a()
      .onTrue(intakeSubsystem.runDeploy(-0.25)))
      .onFalse(intakeSubsystem.stopDeploy());
    //  Hopper
    //TODO
    //  Collector (fine control, variable speed with L/R triggers)
    intakeSubsystem.setDefaultCommand(
      Commands.run(() -> {
          double forward = operatorController.getRightTriggerAxis(); // 0 → 1
          double reverse = operatorController.getLeftTriggerAxis();  // 0 → 1
          double speed = forward - reverse;  // -1 → 1
          double deadband = 0.05;
          if (Math.abs(speed) < deadband) {
              speed = 0;
          }
          intakeSubsystem.spin(speed);
        },
        intakeSubsystem
      )
    );
    // Collector (one-touch at pre-configured speed)
    operatorController.b()
      .onTrue(intakeSubsystem.run())
      .onFalse(intakeSubsystem.stop());

    // Feeder (fine control, variable speed with Left Stick Y-Axis)
    shooterFeeder.setDefaultCommand(
      Commands.run(() -> {
        double speed = -operatorController.getLeftY();
        double deadband = 0.05;
        if (Math.abs(speed) < deadband) {
            speed = 0;
        }
        shooterFeeder.feed(speed);
      }, shooterFeeder)
    );
    // Feeder (one-touch at pre-configured speed)
    operatorController.y()
      .onTrue(shooterFeeder.feed())
      .onFalse(shooterFeeder.stop());

    // Shooter (fine control, variable speed with Right Stick Y-Axis)
    shooter.setDefaultCommand(
      Commands.run(() -> {
          double speed = -operatorController.getRightY();
          double deadband = 0.05;
          if (Math.abs(speed) < deadband) {
              speed = 0;
          }
          shooter.shoot(speed);
        },
        shooter
      )
    );

    // Shooter Hood (one-touch to preset positions)
    operatorController.povRight()
      .onTrue(shooterHood.high());
    operatorController.povLeft()
      .onTrue(shooterHood.low());
    
    // Agitator??
    operatorController.x().whileTrue(agitator.shakeIt());

    //  Combination (one-touch shoot and feed, stop on release)
    operatorController.rightBumper()
        .onTrue(shooter.shoot().andThen(shooterFeeder.feed()))
        .onFalse(shooter.stop().alongWith(shooterFeeder.stop()));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void runRobotPeriodic() {
    shooterPeriodic();
    shooterHoodPeriodic();
    intakeDeployerPeriodic();
    feederPeriodic();
  }

  private void shooterPeriodic() {
    SmartDashboard.putNumber("Left Shooter Speed", shooter.getSpeedLeft().getValueAsDouble());
    SmartDashboard.putNumber("Right Shooter Speed", shooter.getSpeedRight().getValueAsDouble());
  }

  private void shooterHoodPeriodic() {
    SmartDashboard.putNumber("Hood Position", shooterHood.getPosition());
    if (SmartDashboard.getBoolean(FlywheelHood.ResetEncoderDashboardKey, false)) {
      SmartDashboard.putBoolean(FlywheelHood.ResetEncoderDashboardKey, false);
      // Reset the encoder position to 0
      shooterHood.resetEncoder();
    }
  }

  private void intakeDeployerPeriodic() {
    SmartDashboard.putNumber(IntakeSubsystem.IntakeDeployerCurrentPosDashboardKey, intakeSubsystem.getPosition());
    if (SmartDashboard.getBoolean(IntakeSubsystem.ResetEncoderDashboardKey, false)) {
      SmartDashboard.putBoolean(IntakeSubsystem.ResetEncoderDashboardKey, false);
      // Reset the encoder position to 0
      intakeSubsystem.resetEncoder();
    }
  }
  
  private void feederPeriodic() {
    SmartDashboard.putNumber("Feeder Speed", shooterFeeder.getSpeed());
  }
}
