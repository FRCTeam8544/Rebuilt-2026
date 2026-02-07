package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOInputsAutoLogged;
import frc.robot.subsystems.shooter.ShooterIO.ShooterIOInputs;
import frc.robot.subsystems.shooter.ShooterFeedIOInputsAutoLogged;
import frc.robot.subsystems.shooter.ShooterFeedIO.ShooterFeedIOInputs;

public class Shooter extends SubsystemBase{

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

    public void runVoltage(double volts)
    {
        if ((volts >= -12.0) && (volts <= 12.0))
        {
            shooterIO.setVoltage(volts * 12);
            shooterFeedIO.setVoltage(volts * 12);
        }
    }
    public void stop() {
        runVoltage(0.0);
    }
  
  @Override
  public void periodic() {
    shooterIO.updateInputs(shooterInputs);
    shooterFeedIO.updateInputs(shooterFeedInputs);
    Logger.processInputs("Shooter/Motors", shooterInputs);
    Logger.processInputs("Shooter/Feed", shooterFeedInputs);
  }

}