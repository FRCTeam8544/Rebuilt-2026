package frc.robot.subsystems.Feeder;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.subsystems.shooter.Shooter.Flywheel;

public class Feeder extends SubsystemBase{

    // Feed is feed through a 20 to 1 gearbox, but the speed is monitored through the internal
    // encoder
    public static final class FeedWheel {
      public static final double kDriveToOutputGearRatio = 1.0 / 20.0;
      public static final double kOutputToDriveGearRatio = 1.0 / kDriveToOutputGearRatio;
      public static final double kMaxFeedRPM = Constants.NeoVortex.freeSpeedRPM * kDriveToOutputGearRatio;
    }

    private static final int feedMotorCanID = 26;
    private final FeederIO feedIO;
    private final FeedIOInputsAutoLogged feedInputs = new FeedIOInputsAutoLogged();

    // Voltage Feeder tunables
    private final double kNominalFeedFoltage = 10.0;
    private double tuneFeedVoltage = kNominalFeedFoltage;
    private final double tuneFeedVoltStep = 0.25 / Constants.tickUpdatesPerSecond; // 1/4 volt per second

    // --- Suppliers / Triggers ---

    
  public BooleanSupplier isFeeding =
    () -> {
      return feedInputs.wheelVelocity > 1.0; // RPM
    };

   // provide current roller wheel RPM
   public DoubleSupplier rollerRpmSupplier = 
      () -> {
        return feedInputs.wheelVelocity;
      };

   
    public Feeder()
    {
      this.feedIO = new FeederIOFlex(feedMotorCanID);

      setupDefaultDashboard();
    }


    public void tuneIncreaseFeedVoltage() {
      tuneFeedVoltage += tuneFeedVoltStep;
      if (tuneFeedVoltage > Constants.kNominalVoltage) {
        tuneFeedVoltage = Constants.kNominalVoltage;
      }
      else if (tuneFeedVoltage < 0.0) {
        tuneFeedVoltage = 0.0;
      }
    }

    public void tuneDecreaseFeedVoltage() {
      tuneFeedVoltage -= tuneFeedVoltStep;
      if (tuneFeedVoltage < 0.0) {
        tuneFeedVoltage = 0.0;
      } else if (tuneFeedVoltage > Constants.kNominalVoltage) {
        tuneFeedVoltage = Constants.kNominalVoltage;
      }
    }

    public void runOpenLoop()
    {
      runOpenLoop( tuneFeedVoltage / Constants.Neo550.nominalVoltage);
    }

    public void runOpenLoopReverse()
    {
      runOpenLoop( -1 * (tuneFeedVoltage / Constants.Neo550.nominalVoltage));
    }

    public void runOpenLoop(double duty)
    {
      double adjustedDuty = duty;

      if (adjustedDuty > 1.0) {
        adjustedDuty = 1.0;
      }
      else if (adjustedDuty < -1.0) {
        adjustedDuty = -1.0;
      }

      double scaledVolts = duty * Constants.kNominalVoltage;
      feedIO.setVoltage(scaledVolts);

      feedInputs.voltageSetPoint = scaledVolts;
      feedInputs.velocitySetPoint = 0.0;
    }

    public void resetTuneDefaultVoltage() {
      tuneFeedVoltage = kNominalFeedFoltage;
    }

    // -------------------  FEED --------------------------------

    public void runAtRpm(double rpm)
    {
      double adjustedRpm = rpm;
      // Limit wheel RPM while preserving direction
      if (Math.abs(rpm) > FeedWheel.kMaxFeedRPM)
      {
        adjustedRpm = Math.copySign(FeedWheel.kMaxFeedRPM, adjustedRpm);
      }

      // Prevent out of spec RPM
      adjustedRpm = Math.min(adjustedRpm,FeedWheel.kMaxFeedRPM);
      if (adjustedRpm < 0) {
        adjustedRpm = 0.0;
      }

      feedInputs.voltageSetPoint = 0.0;
      feedInputs.velocitySetPoint = adjustedRpm * Flywheel.kOutputToDriveGearRatio;

      feedIO.setVelocity(feedInputs.velocitySetPoint);
    }

    public void stopMotors() {
      
      feedInputs.voltageSetPoint = 0.0;
      feedInputs.velocitySetPoint = 0.0;
      feedIO.setVoltage(0);
    }
  
  @Override
  public void periodic() {
    feedIO.updateInputs(feedInputs);
    Logger.processInputs("Feeder/Feed", feedInputs);
    
    
    SmartDashboard.putNumber("Feeder RPM", feedInputs.motorVelocity);
    SmartDashboard.putNumber("Feeder RPM Setpoint", feedInputs.velocitySetPoint);
    SmartDashboard.putNumber("Feeder Volts Setpoint", feedInputs.voltageSetPoint);
    SmartDashboard.putNumber("Feeder Leader Temp", feedInputs.motorTemperature);
  }

  private void setupDefaultDashboard()
  {
    SmartDashboard.setDefaultNumber("Feeder RPM", feedInputs.motorVelocity);
    SmartDashboard.setDefaultNumber("Feeder RPM Setpoint", feedInputs.velocitySetPoint);
    SmartDashboard.setDefaultNumber("Feeder Volts Setpoint", feedInputs.voltageSetPoint);
    SmartDashboard.setDefaultNumber("Feeder Motor Temp", feedInputs.motorTemperature);
  }

}
