package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.Feeder.Feeder;


/**
 * Spins up the shooter to a target RPM, then feeds fuel once at speed. Terminates after the feeder
 * has run for {@link #FEED_DURATION_SECONDS} once the flywheel reaches its RPM target.
 */
public class ShootFuelCommand extends Command {

  private static final double FEED_DURATION_SECONDS = 1.0;
  private final int feederRpmCmd = 300;

  private Shooter shooter;
  private Feeder feeder;
  private DoubleSupplier rpmTargetSupplier;
  private BooleanSupplier atRpmBooleanSupplier;

  private boolean fuelFired = false;
  private double feedStartTimestamp = 0.0;


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

  @Override
  public void initialize() {
    fuelFired = false;
    feedStartTimestamp = 0.0;
  }

  @Override
  public void execute() {
    shooter.runAtRpm(rpmTargetSupplier.getAsDouble());

    if (atRpmBooleanSupplier.getAsBoolean()) {
      if (feedStartTimestamp == 0.0) {
        feedStartTimestamp = Timer.getFPGATimestamp();
      }
      feeder.runAtRpm(feederRpmCmd);
      if (Timer.getFPGATimestamp() - feedStartTimestamp >= FEED_DURATION_SECONDS) {
        fuelFired = true;
      }
    } else {
      feeder.stopMotors();
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stopMotors();
    feeder.stopMotors();
  }

  @Override
  public boolean isFinished() {
    return fuelFired;
  }
}