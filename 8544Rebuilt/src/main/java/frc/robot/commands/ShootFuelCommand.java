package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.Feeder.Feeder;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootFuelCommand extends Command {

  private final int feederRpmCmd = 300;

  private Shooter shooter;
  private Feeder feeder;
  private DoubleSupplier rpmTargetSupplier;
  private BooleanSupplier atRpmBooleanSupplier;


  public ShootFuelCommand(Shooter shooter, Feeder feeder,
                          double fixedRpm)
  {
    this.shooter = shooter;
    this.feeder = feeder;
    this.rpmTargetSupplier = () -> { return fixedRpm; };
    this.atRpmBooleanSupplier = shooter.flywheelAtRpmTarget;

    addRequirements(shooter);
    addRequirements(feeder);
    setName("ShootingFuelFixedRpm");
  }

  public ShootFuelCommand(Shooter shooter, Feeder feeder,
                          DoubleSupplier rpmDoubleSupplier) {
    this.shooter = shooter;
    this.feeder = feeder;
    this.rpmTargetSupplier = rpmDoubleSupplier;
    this.atRpmBooleanSupplier = shooter.flywheelAtRpmTarget;
    
    addRequirements(shooter);
    addRequirements(feeder);
    setName("ShootingFuelAdjustedRpm");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.runAtRpm(rpmTargetSupplier.getAsDouble());
    
    if (atRpmBooleanSupplier.getAsBoolean())
    {
        feeder.runAtRpm(feederRpmCmd);
    }
    else
    {
        feeder.stopMotors();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
     shooter.stopMotors();
     feeder.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}