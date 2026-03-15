package frc.robot.subsystems.shooter;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Shooter extends SubsystemBase{

    // Limit chonker flywheel.. so limit to be safe for now
    public static final class Flywheel {
      public static final double kMaxShooterRPM = 4000; // Output flywheel, not motor
      public static final double kDriveToOutputGearRatio = 1.4;
      public static final double kOutputToDriveGearRatio = 1.0 / kDriveToOutputGearRatio;
    }

    public static final int leftMotorCanID = 24;
    public static final int rightMotorCanID = 25;
    public static final int feedMotorCanID = 26;

    private final ShooterIO shooterIO;
    private final ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();

    private double tuneShootVoltage = 3.0;
    private final double tuneShootVoltStep = 0.25 / Constants.tickUpdatesPerSecond; // 1/4 volt per second

   private double tuneShootRpmAdjust = 0.0;

   private double rpmAtSpeedTolerance = 50;

   // ---- Suppliers / Triggers ----

   public BooleanSupplier flywheelAtRpmTarget =
   () -> {
     if (shooterInputs.flywheelVelocitySetPoint > 0) {
      return MathUtil.isNear(shooterInputs.flywheelVelocitySetPoint,
                             shooterInputs.flywheelMeidanVelocity,
                             rpmAtSpeedTolerance);
     }
     else {
      return false;
     }
   };

   public DoubleSupplier rpmSetPointSupplier = 
    () -> {
      return shooterInputs.flywheelVelocitySetPoint;
    };
    
  public DoubleSupplier flywheelRpmSupplier =
    () -> {
      return shooterInputs.flywheelVelocity;
    };

  public DoubleSupplier motorTemperatureSupplier =
    () -> {
      return Math.max(shooterInputs.leaderMotorTemperature, shooterInputs.followMotorTemperature);
    };


  public BooleanSupplier flywheelAutoToggleBooleanSupplier =
  () -> {
    return shooterInputs.flywheelAutoRPMToggle;

  };

   // Shooter
    public Shooter(ShooterIO shooterIO)
    {
      this.shooterIO = shooterIO;

      setupDefaultDashboard();
    }

    // Provide the shooter velocity as revolutions per second
    public double getShooterFFCharacterizationVelocity() {
      return shooterInputs.motorVelocity / 60.0; // Convert RPM to RPS
    }

    public void tuneIncreaseShootVoltage() {
      tuneShootVoltage += tuneShootVoltStep;
      if (tuneShootVoltage > Constants.kNominalVoltage) {
        tuneShootVoltage = Constants.kNominalVoltage;
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
      runOpenLoop( tuneShootVoltage / Constants.kNominalVoltage );
    }

    public void runOpenLoop(double duty)
    {
      double adjustedDuty = duty;

      // Prevent duty beyond 1 to 0
      adjustedDuty = Math.min(adjustedDuty, 1.0);
      if (adjustedDuty < 0.0)
      {
        adjustedDuty = 0.0;
      }

      // Safety limit RPM
      if (isFlywheelOverspeed()) {
         adjustedDuty = 0.0; // Bring flywheel speed down
      }

      double scaledVolts = adjustedDuty * Constants.KrakenX60.nominalVoltage;
      shooterIO.setVoltage(scaledVolts);

      shooterInputs.voltageSetPoint = scaledVolts;
      shooterInputs.flywheelVelocitySetPoint = 0.0;
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
      shooterInputs.flywheelVelocitySetPoint = 0;
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
      
      // Safety limit RPM
      if (isFlywheelOverspeed()) {
         shooterIO.setVoltage(0); // Disable PID loop and null voltage
         shooterInputs.flywheelVelocitySetPoint = 0.0;
         shooterInputs.voltageSetPoint = 0.0;
      }

      // Scale requested flywheel RPM to shooter motor RPM
      shooterInputs.flywheelVelocitySetPoint = adjustedRpm;
      shooterInputs.voltageSetPoint = 0.0;
      
      // This will decrease the requested motor RPM so that the output flywheel is at the requested rpm.
      final double motorVelocitySetPoint = shooterInputs.flywheelVelocitySetPoint * Flywheel.kOutputToDriveGearRatio;
      shooterIO.setVelocity(motorVelocitySetPoint);
    }

  public boolean isFlywheelOverspeed() {

      // Safety limit RPM
      if (shooterInputs.flywheelVelocity > Flywheel.kMaxShooterRPM) {
        shooterInputs.maxFlywheelSpeedHit = true;
      }
      // After overspeed event, only allow use when flywheel is well below max speed to avoid pulsing the wheel
      else if ((shooterInputs.maxFlywheelSpeedHit) &&
               (shooterInputs.flywheelVelocity > Flywheel.kMaxShooterRPM - 500)) {
        shooterInputs.maxFlywheelSpeedHit = false;
      }

      return shooterInputs.maxFlywheelSpeedHit;
  }


  @Override
  public void periodic() {
    shooterIO.updateInputs(shooterInputs);
    Logger.processInputs("Shooter", shooterInputs);
    
    
    SmartDashboard.putNumber("Shooter RPM", shooterInputs.flywheelVelocity);
    SmartDashboard.putNumber("Shooter RPM Setpoint", shooterInputs.flywheelVelocitySetPoint);
    
    SmartDashboard.putNumber("Shooter Leader Temp", shooterInputs.leaderMotorTemperature);
    SmartDashboard.putNumber("Shooter Follow Temp", shooterInputs.followMotorTemperature);

    shooterInputs.flywheelAutoRPMToggle = 
         SmartDashboard.getBoolean("AutoShootRPMToggle", shooterInputs.flywheelAutoRPMToggle);
  }

  private void setupDefaultDashboard()
  {
    // Dashboard user settings
    SmartDashboard.setDefaultBoolean("AutoShootRPMToggle", shooterInputs.flywheelAutoRPMToggle);

    // Dashboard display
    SmartDashboard.setDefaultNumber("Shooter RPM", shooterInputs.flywheelVelocity);
    SmartDashboard.setDefaultNumber("Shooter RPM Setpoint", shooterInputs.flywheelVelocitySetPoint);
    SmartDashboard.setDefaultNumber("Shooter Leader Temp", shooterInputs.leaderMotorTemperature);
    SmartDashboard.setDefaultNumber("Shooter Follow Temp", shooterInputs.followMotorTemperature);
  }

}