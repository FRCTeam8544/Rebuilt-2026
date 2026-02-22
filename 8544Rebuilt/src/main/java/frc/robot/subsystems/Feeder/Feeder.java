package frc.robot.subsystems.Feeder;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOInputsAutoLogged;
import frc.robot.subsystems.shooter.ShooterIO.ShooterIOInputs;
import frc.robot.Constants;
import frc.robot.subsystems.Feeder.*;;
public class Feeder extends SubsystemBase{



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

    private final FeederIO feedIO;
    private final FeedIOInputsAutoLogged feedInputs = new FeedIOInputsAutoLogged();

    private double tuneFeedVoltage = 10.0;
    private double tuneShootVoltage = 0.0;
    private final double tuneFeedVoltStep = 1.0 / 50.0; // 1 volt per second
    private final double tuneShootVoltStep = 0.25 / 50; // 1/4 volt per second

   private double tuneShootRpmAdjust = 0.0;
   private double tuneFeedRpmAdjust = 0.0;
   
    public Feeder()
    {
      this.feedIO = new FeederIOFlex(feedMotorCanID);

      setupDefaultDashboard();
    }


    public void tuneIncreaseFeedVoltage() {
      tuneFeedVoltage += tuneFeedVoltStep;
      if (tuneFeedVoltage > 12.0) {
        tuneFeedVoltage = 12.0;
      }
      else if (tuneFeedVoltage < 0.0) {
        tuneFeedVoltage = 0.0;
      }
    }

    public void tuneDecreaseFeedVoltage() {
      tuneFeedVoltage -= tuneFeedVoltStep;
      if (tuneFeedVoltage < 0.0) {
        tuneFeedVoltage = 0.0;
      } else if (tuneFeedVoltage > 12.0) {
        tuneFeedVoltage = 12.0;
      }
    }


    public void runFeedOpenLoop()
    {
      runFeedOpenLoop( tuneFeedVoltage / Constants.Neo550.nominalVoltage);
    }


    public void runFeedOpenLoop(double duty)
    {
      double adjustedDuty = duty;

    /*  if (adjustedDuty != 0.0) {
        adjustedDuty += tuneFeedVoltage / Constants.NeoVortex.nominalVoltage;
      }*/

       // Prevent duty beyond 1 to 0
      adjustedDuty = Math.min(adjustedDuty, 1.0);
      if ( adjustedDuty < 0.0) {
        adjustedDuty = 0.0;
      }

      // Prevent out of spec RPM
     // if (Math.abs(feedInputs.wheelVelocity) > FeedWheel.kMaxFeedRPM) {
      //////  adjustedDuty = 0.0;
      //}

      double scaledVolts = duty * Constants.NeoVortex.nominalVoltage;
      feedIO.setVoltage(scaledVolts);

      feedInputs.voltageSetPoint = scaledVolts;
      feedInputs.velocitySetPoint = 0.0;
    }

    public void resetFeedDefaultVoltage() {
      tuneFeedVoltage = 0.0;
    }

    public void resetFeedDefaultRpm() {
      tuneFeedRpmAdjust = 0.0;
    }

    public void resetShooterDefaultVoltage() {
      tuneShootVoltage = 0.0;
    }

    public void resetShooterDefaultRpm() {
      tuneShootRpmAdjust = 0.0;
    }

    public void feedRpmAdjust(double rpmAdjust) {
      tuneFeedRpmAdjust += rpmAdjust;
    }

    public void shooterRpmAdjust(double rpmAdjust)
    {
      tuneShootRpmAdjust += rpmAdjust;
    }

    // ------------------ Shooter ------------------------

    public void stopShooter() {
    //  if (Math.abs(getFlywheelVelocityRPM()) > 250) {
     //   shooterIO.setVoltage(-1.0); // Gentle break
     // }
     // else {
     
      feedInputs.voltageSetPoint = 0.0;
      feedInputs.velocitySetPoint = 0;
        feedIO.setVoltage(0.0); // Gentle break
     // }
    }

    // This should be the requested flywheel RPM
    public void runShooter(double rpm) {

      double adjustedRpm = rpm;
      
      if (rpm > 0.0) {
        adjustedRpm += tuneShootRpmAdjust;
      }

      // Prevent out of spec RPM

    }

    // -------------------  FEED --------------------------------

    public void runFeed(double rpm)
    {
      double adjustedRpm = rpm;
      if (rpm > 0.0)
      {
        adjustedRpm += tuneFeedRpmAdjust;
      }

      // Prevent out of spec RPM
      adjustedRpm = Math.min(adjustedRpm,FeedWheel.kMaxFeedRPM);
      if (adjustedRpm < 0) {
        adjustedRpm = 0.0;
      }

      feedInputs.voltageSetPoint = 0.0;
      feedInputs.velocitySetPoint = adjustedRpm;

      feedIO.setVelocity(feedInputs.velocitySetPoint);
    }

    public void stopFeed() {
      
      feedInputs.voltageSetPoint = 0.0;
      feedInputs.velocitySetPoint = 0.0;
      feedIO.setVoltage(0);
    }
  
  @Override
  public void periodic() {
    feedIO.updateInputs(feedInputs);
    Logger.processInputs("Shooter/Feed", feedInputs);
    
    
    SmartDashboard.putNumber("Shooter RPM", feedInputs.motorVelocity);
    SmartDashboard.putNumber("Shooter RPM Setpoint", feedInputs.velocitySetPoint);
    
    SmartDashboard.putNumber("Shooter Leader Temp", feedInputs.motorTemperature);

  }

  private void setupDefaultDashboard()
  {
    SmartDashboard.setDefaultNumber("Shooter RPM", feedInputs.motorVelocity);
    SmartDashboard.setDefaultNumber("Shooter RPM Setpoint", feedInputs.velocitySetPoint);
    SmartDashboard.setDefaultNumber("Shooter Leader Temp", feedInputs.motorTemperature);
  }

}
