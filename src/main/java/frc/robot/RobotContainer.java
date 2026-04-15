// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DashboardConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.April;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystemCollector;
import frc.robot.subsystems.Swerve;
//import frc.robot.subsystems.Intake.IntakeCollector;
//import frc.robot.subsystems.Intake.IntakeDeploy;
import frc.robot.subsystems.Shooter.Agitator;
import frc.robot.subsystems.Shooter.Feeder;
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
  //private final IntakeDeploy intakeDeploy = new IntakeDeploy();
  //private final IntakeCollector intakeCollector = new IntakeCollector();
  private final Agitator agitator = new Agitator();
  private final Feeder shooterFeeder = new Feeder();
  private final FlywheelShooter shooter = new FlywheelShooter();
  private final IntakeSubsystemCollector intakeCollector = new IntakeSubsystemCollector();

  // Commands
  private final CommandXboxController driverController =
  
    new CommandXboxController(OperatorConstants.DriverControllerPort);
  private static final CommandXboxController operatorController =
    new CommandXboxController(OperatorConstants.OperatorControllerPort);

  /* Path follower */
  private final SendableChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public static CommandXboxController getOperatorController() {
    return operatorController;
  }
  public RobotContainer() {
    registerDashboardProperties();

    registerNamedCommands();
    autoChooser = AutoBuilder.buildAutoChooser("Just Shoot");
    SmartDashboard.putData(DashboardConstants.AutoModeKey, autoChooser);
    
    configureBindings();
    // Add camera feed to dashboard
    CameraServer.startAutomaticCapture();
  }

  private void registerNamedCommands() {
    NamedCommands.registerCommand("shoot", shooter.shootWithPID());
    NamedCommands.registerCommand("shootStop", shooter.stop());
    NamedCommands.registerCommand("stopIntakeDeploy", intakeSubsystem.stopDeploy());
    NamedCommands.registerCommand("stopIntake", intakeCollector.stop());
    NamedCommands.registerCommand("deployIntake", intakeSubsystem.runDeploy(-1));
    NamedCommands.registerCommand("retractIntake", intakeSubsystem.runDeploy(1));
    NamedCommands.registerCommand("runIntake", intakeCollector.run());
    NamedCommands.registerCommand("agitate", agitator.run());
    NamedCommands.registerCommand("feed", shooterFeeder.feed());
    NamedCommands.registerCommand("shooterFeedStop", shooterFeeder.stop());
    NamedCommands.registerCommand("agitateStop", agitator.stop());


    // Add more commands here as needed
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
    swerveDrivetrain.configureBindings(driverController, operatorController);
  }
  private void configureGameplayBindings() {
    //TODO - refactor into methods
    //  Deploy/Retract Intake
  
    
    //  Intake Collector (variable speed with L/R triggers)
    intakeSubsystem.setDefaultCommand(
      Commands.run(() -> {
          double forward = operatorController.getRightTriggerAxis(); // 0 → 1
          double reverse = operatorController.getLeftTriggerAxis();  // 0 → 1s
          double speed = forward - reverse;  // -1 → 1
          double deadband = 0.05;
          if (Math.abs(speed) < deadband) {
              speed = 0;
          }
          intakeCollector.spin();
        },
        intakeSubsystem
      )
    );

    // Shooter (one-touch at pre-configured speed)
    /*
    operatorController.b()
      .onTrue(shooter.shoot())
      .onFalse(shooter.stop());
    */
    // Feeder (variable speed with Left Stick Y-Axis)
//     shooterFeeder.setDefaultCommand(
//       Commands.run(() -> {
//         double speed = -operatorController.getLeftY();
//         double deadband = 0.05;
//         if (Math.abs(speed) < deadband) {
//             speed = 0;
//         }
//         shooterFeeder.feed();
//       }, shooterFeeder)
//     );

//     // Shooter (variable speed with Right Stick Y-Axis)
//     shooter.setDefaultCommand(
//     Commands.run(() -> {
//         double speed = -operatorController.getRightY();
//         if (Math.abs(speed) < 0.05) speed = 0;
//         shooter.setSpeed(speed);
//     }, shooter)
// );

    // Shooter Hood (one-touch to preset positions)
    operatorController.leftBumper()
         .onTrue(intakeCollector.spin()
             .alongWith(agitator.shakeIt())
         
             )
             .onFalse(agitator.stop().alongWith(intakeCollector.stop()));

    operatorController.rightBumper()
      .onTrue(shooter.shootWithPID()
        .alongWith(shooterFeeder.feed())
       )
        .onFalse(shooter.stop().alongWith(shooterFeeder.stop()));
      
      operatorController.leftTrigger()
        .onTrue(shooter.shoot(-10)
        .alongWith(shooterFeeder.feed())
        
      )
        //30000 LT Passing
        .onFalse(shooter.stop().alongWith(shooterFeeder.stop()));
      
        operatorController.rightTrigger()
        .onTrue(shooter.shoot(-1.13)
        .alongWith(shooterFeeder.feed())
      )
        //3400 (Trench) RT 
        .onFalse(shooter.stop().alongWith(shooterFeeder.stop()));
     
     
     
      operatorController.x()
     .onTrue(intakeCollector.spin())
     .onFalse(intakeCollector.stop());
    // Intake deploy/retract
    operatorController.y() // retract
          .onTrue(intakeSubsystem.runDeploy(.5))
          .onFalse(intakeSubsystem.stopDeploy());

    operatorController.a() // deploy
      .onTrue(intakeSubsystem.runDeploy(-0.32))
      .onFalse(intakeSubsystem.stopDeploy());

    //operatorController.b().whileTrue();

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
    intakeDeployerPeriodic();
    feederPeriodic();
    agitatorPeriodic();
  }
  
  
  
  private void shooterPeriodic() {
    SmartDashboard.putNumber("Left Shooter Speed", shooter.getSpeedLeft().getValueAsDouble());
    SmartDashboard.putNumber("Right Shooter Speed", shooter.getSpeedRight().getValueAsDouble());
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

  private void agitatorPeriodic() {
    SmartDashboard.putNumber("Agitator Speed", agitator.getSpeed());
  }
  
}
