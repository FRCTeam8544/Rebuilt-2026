package frc.robot.subsystems.shooter;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOInputsAutoLogged;
import frc.robot.subsystems.shooter.ShooterIO.ShooterIOInputs;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.FeedIOInputsAutoLogged;
import frc.robot.subsystems.shooter.FeedIO.FeedIOInputs;

public class Shooter extends SubsystemBase{

    // Limit chonker flywheel.. so limit to be safe for now
    public static final class Flywheel {
      public static final double kMaxShooterRPM = 4000; // Output flywheel, not motor
      public static final double kDriveToOutputGearRatio = 1.4;
      public static final double kOutputToDriveGearRatio = 1.0 / kDriveToOutputGearRatio;
    }

    // Feed is feed through a 20 to 1 gearbox, but the speed is monitored through the internal
    // encoder
    public static final class FeedWheel {
      public static final double kDriveToOutputGearRatio = 1.0 / 20.0;
      public static final double kOutputToDriveGearRatio = 1.0 / kDriveToOutputGearRatio;
      public static final double kMaxFeedRPM = Constants.NeoVortex.freeSpeedRPM * kDriveToOutputGearRatio;
    }

    public static final int leftMotorCanID = 24;
    public static final int rightMotorCanID = 25;
    public static final int feedMotorCanID = 26;

    private final ShooterIO shooterIO;
    private final ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();

    private final FeedIO feedIO;
    private final FeedIOInputsAutoLogged feedInputs = new FeedIOInputsAutoLogged();

    private double tuneFeedVoltage = 10.0;
    private double tuneShootVoltage = 0.0;
    private final double tuneFeedVoltStep = 1.0 / 50.0; // 1 volt per second
    private final double tuneShootVoltStep = 0.25 / 50; // 1/4 volt per second

   private double tuneShootRpmAdjust = 0.0;
   private double tuneFeedRpmAdjust = 0.0;

   // ---- Suppliers / Triggers ----

  public DoubleSupplier flywheelRpmSupplier =
    () -> {
      return shooterInputs.flywheelVelocity;
    };


   // Shooter
    public Shooter()
    {
      this.shooterIO = new ShooterIOTalonFX(leftMotorCanID, rightMotorCanID);
      this.feedIO = new FeedIOFlex(feedMotorCanID);

      setupDefaultDashboard();
    }

    // Provide the shooter velocity as revolutions per second
    public double getShooterFFCharacterizationVelocity() {
      return shooterInputs.motorVelocity / 60.0; // Convert RPM to RPS
    }

    public void tuneIncreaseShootVoltage() {
      tuneShootVoltage += tuneShootVoltStep;
      if (tuneShootVoltage > 12.0) {
        tuneShootVoltage = 12.0;
      }
    }

    public void tuneDecreaseShootVoltage() {
      tuneShootVoltage -= tuneShootVoltStep;
      if (tuneShootVoltage < 0.0) {
        tuneShootVoltage = 0.0;
      }
    }

    public void runOpenLoop()
    {
      runShooterOpenLoop( tuneShootVoltage / Constants.KrakenX60.nominalVoltage ); // Will be adjusted internally
    }

    public void runOpenLoop()
    {
      runFeedOpenLoop( tuneFeedVoltage / Constants.Neo550.nominalVoltage);
    }

    public void runOpenLoop(double duty)
    {
      double adjustedDuty = duty;
      
    /*  if (adjustedDuty > 0.0) {
        adjustedDuty += tuneShootVoltage / Constants.NeoVortex.nominalVoltage;
      }*/

      // Prevent duty beyond 1 to 0
      adjustedDuty = Math.min(adjustedDuty, 1.0);
      if (adjustedDuty < 0.0)
      {
        adjustedDuty = 0.0;
      }

      // Safety limit RPM
      if (Math.abs(getFlywheelVelocityRPM()) > Flywheel.kMaxShooterRPM) {
        shooterInputs.maxFlywheelSpeedHit = true;
        adjustedDuty = 0.0;
      }
      else {
        shooterInputs.maxFlywheelSpeedHit = false;
      }

      double scaledVolts = adjustedDuty * Constants.KrakenX60.nominalVoltage;
      shooterIO.setVoltage(scaledVolts);

      shooterInputs.voltageSetPoint = scaledVolts;
      shooterInputs.velocitySetPoint = 0.0;
    }

    public void resetShooterDefaultVoltage() {
      tuneShootVoltage = 0.0;
    }

    public void resetShooterDefaultRpm() {
      tuneShootRpmAdjust = 0.0;
    }

    public void shooterRpmAdjust(double rpmAdjust)
    {
      tuneShootRpmAdjust += rpmAdjust;
    }

    public void stopMotors() {
     
// TODO Add gentle break? or use seperate command?
      shooterInputs.voltageSetPoint = 0.0;
      shooterInputs.velocitySetPoint = 0;
      shooterIO.setVoltage(0.0);
    }

    // This should be the requested flywheel RPM
    public void runAtRpm(double rpm) {

      double adjustedRpm = rpm;
      
      if (rpm > 0.0) {
        adjustedRpm += tuneShootRpmAdjust;
      }

      // Prevent out of spec RPM
      adjustedRpm = Math.min(adjustedRpm,Flywheel.kMaxShooterRPM);
      if (adjustedRpm < 0)
      {
        adjustedRpm = 0;
      }

      // Scale requested flywheel RPM to shooter motor RPM
      shooterInputs.voltageSetPoint = 0.0;
      shooterInputs.velocitySetPoint = adjustedRpm * Flywheel.kOutputToDriveGearRatio;

      shooterIO.setVelocity(shooterInputs.velocitySetPoint);
    }
  
  @Override
  public void periodic() {
    shooterIO.updateInputs(shooterInputs);
    Logger.processInputs("Shooter", shooterInputs);
    
    
    SmartDashboard.putNumber("Shooter RPM", shooterInputs.flywheelVelocity);
    SmartDashboard.putNumber("Shooter RPM Setpoint", shooterInputs.velocitySetPoint);
    
    SmartDashboard.putNumber("Shooter Leader Temp", shooterInputs.leaderMotorTemperature);
    SmartDashboard.putNumber("Shooter Follow Temp", shooterInputs.followMotorTemperature);

  }

  private void setupDefaultDashboard()
  {
    SmartDashboard.setDefaultNumber("Shooter RPM", shooterInputs.flywheelVelocity);
    SmartDashboard.setDefaultNumber("Shooter RPM Setpoint", shooterInputs.velocitySetPoint);
    SmartDashboard.setDefaultNumber("Shooter Leader Temp", shooterInputs.leaderMotorTemperature);
    SmartDashboard.setDefaultNumber("Shooter Follow Temp", shooterInputs.followMotorTemperature);
  }

}