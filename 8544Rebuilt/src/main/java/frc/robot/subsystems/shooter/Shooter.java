package frc.robot.subsystems.shooter;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

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

   // ---- Suppliers / Triggers ----

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

   // Shooter
    public Shooter()
    {
      this.shooterIO = new ShooterIOTalonFX(leftMotorCanID, rightMotorCanID);

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
      shooterInputs.voltageSetPoint = 0.0;
      shooterInputs.flywheelVelocitySetPoint = adjustedRpm;
      final double motorVelocitySetPoint = shooterInputs.flywheelVelocitySetPoint * Flywheel.kOutputToDriveGearRatio;

      shooterIO.setVelocity(motorVelocitySetPoint);
    }

// --- ADDED FOR LED CONTROL DURING SHOOTING ---
    // Minimum motor RPM setpoint to be considered "spinning up"
    private static final double FLYWHEEL_SETPOINT_MIN_MOTOR_RPM = 100;
    // Motor RPM tolerance for "at setpoint" (launch-ready)
    private static final double LAUNCH_TOLERANCE_MOTOR_RPM = 150;

    /**
     * Returns true when the flywheel is at its commanded velocity setpoint within tolerance,
     * indicating the robot is ready to launch.
     */
    public boolean isAtLaunchSetpoint() {
      return shooterInputs.velocitySetPoint > FLYWHEEL_SETPOINT_MIN_MOTOR_RPM
          && Math.abs(shooterInputs.motorVelocity - shooterInputs.velocitySetPoint)
              < LAUNCH_TOLERANCE_MOTOR_RPM;
    }

    /**
     * Returns true when the flywheel is spinning and the feed is actively running,
     * indicating the robot is actively launching.
     */
    public boolean isShooting() {
      boolean feedActive = feedInputs.voltageSetPoint > 0.5 || feedInputs.velocitySetPoint > 50;
      return shooterInputs.flywheelVelocity > 100 && feedActive;
    }
// --- END  ---
  
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

  }

  private void setupDefaultDashboard()
  {
    SmartDashboard.setDefaultNumber("Shooter RPM", shooterInputs.flywheelVelocity);
    SmartDashboard.setDefaultNumber("Shooter RPM Setpoint", shooterInputs.flywheelVelocitySetPoint);
    SmartDashboard.setDefaultNumber("Shooter Leader Temp", shooterInputs.leaderMotorTemperature);
    SmartDashboard.setDefaultNumber("Shooter Follow Temp", shooterInputs.followMotorTemperature);
  }

}