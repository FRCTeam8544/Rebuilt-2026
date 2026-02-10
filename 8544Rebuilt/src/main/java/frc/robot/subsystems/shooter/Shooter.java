package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOInputsAutoLogged;
import frc.robot.subsystems.shooter.ShooterIO.ShooterIOInputs;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterFeedIOInputsAutoLogged;
import frc.robot.subsystems.shooter.ShooterFeedIO.ShooterFeedIOInputs;

public class Shooter extends SubsystemBase{

    // Neo vortex can do over 5000 RPM, but flywheel is quite a chonker... so limit to be safe for now
    public static final double kMaxShooterRPM = 2000;
    public static final double kMaxFeedRPM = 6000; // Attached to 20 to 1 gearbox

    public static final int leftMotorCanID = 24;
    public static final int rightMotorCanID = 25;
    public static final int feedMotorCanID = 26;

    private final ShooterIO shooterIO;
    private final ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();

    private final ShooterFeedIO shooterFeedIO;
    private final ShooterFeedIOInputsAutoLogged shooterFeedInputs = new ShooterFeedIOInputsAutoLogged();

    public Shooter()
    {
       this.shooterIO = new ShooterIOFlex(leftMotorCanID, rightMotorCanID);
       this.shooterFeedIO = new ShooterFeedIOFlex(feedMotorCanID);
    }

    public void runShooterOpenLoop(double duty)
    {
      // Prevent duty beyond 1 to -1
      if ( (duty > 1.0) || (duty < -1.0) )
      {
        duty = Math.copySign(1.0, duty);
      }

      double scaledVolts = duty * Constants.NeoVortex.nominalVoltage;
      shooterIO.setVoltage(scaledVolts);

      shooterInputs.voltageSetPoint = scaledVolts;
      shooterInputs.velocitySetPoint = 0.0;
    }

    public void runFeedOpenLoop(double duty)
    {
       // Prevent duty beyond 1 to -1
      if ( (duty > 1.0) || (duty < -1.0) )
      {
        duty = Math.copySign(1.0, duty);
      }

      double scaledVolts = duty * Constants.NeoVortex.nominalVoltage;
      shooterFeedIO.setVoltage(scaledVolts);

      shooterFeedInputs.voltageSetPoint = scaledVolts;
      shooterFeedInputs.velocitySetPoint = 0.0;
    }

    public void runShooter(double rpm) {
      // Prevent out of spec RPM
      if ( (rpm > kMaxShooterRPM) || (rpm < -kMaxShooterRPM) )
      {
        rpm = Math.copySign(kMaxShooterRPM, rpm);
      }
      shooterInputs.velocitySetPoint = rpm * kMaxShooterRPM;
      shooterIO.setVelocity(shooterInputs.velocitySetPoint);
    }

    public void runFeed(double rpm)
    {
       // Prevent out of spec RPM
      if ( (rpm > kMaxFeedRPM) || (rpm < -kMaxFeedRPM) )
      {
        rpm = Math.copySign(kMaxShooterRPM, rpm);
      }

      shooterFeedInputs.voltageSetPoint = 0.0;
      shooterFeedInputs.velocitySetPoint = rpm;

      shooterFeedIO.setVelocity(shooterFeedInputs.velocitySetPoint);
    }

    public void stopOpenLoop() {
        runFeedOpenLoop(0.0);
        runShooterOpenLoop(0.0);
    }
  
  @Override
  public void periodic() {
    shooterIO.updateInputs(shooterInputs);
    shooterFeedIO.updateInputs(shooterFeedInputs);
    Logger.processInputs("Shooter/Motors", shooterInputs);
    Logger.processInputs("Shooter/Feed", shooterFeedInputs);
  }

}