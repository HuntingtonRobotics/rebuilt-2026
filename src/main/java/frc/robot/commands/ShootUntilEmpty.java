//Tm Bennett/Hunter

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Shooter.Agitator;
import frc.robot.subsystems.Shooter.Feeder;
import frc.robot.subsystems.Shooter.FlywheelShooter;

/** Composite command for shooting the Fuel until empty or command is disabled */
public class ShootUntilEmpty extends Command {
  private final Agitator agitator;
  private final Feeder feeder;
  private final FlywheelShooter shooter;
  private final IntakeSubsystem intake;

  public ShootUntilEmpty(Agitator agitator, Feeder feeder, FlywheelShooter shooter, IntakeSubsystem intake) {
    this.agitator = agitator;
    this.feeder = feeder;
    this.shooter = shooter;
    this.intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(agitator, feeder, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Commands.parallel(
      Commands.sequence(
        shooter.shoot(),
        feeder.feed()
      ),
      agitator.shakeIt()
    );
    try {
      wait(200);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }
    

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Commands.parallel(
      shooter.stop(),
      feeder.stop(),
      agitator.stop()
    );
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
