package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOInputsAutoLogged;
import frc.robot.subsystems.shooter.ShooterIO.ShooterIOInputs;

public class Shooter extends SubsystemBase{

    public static final int leftMotorCanID = 24;
    public static final int rightMotorCanID = 25;

    private final ShooterIO shooterIO;
    private final ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();

    public Shooter()
    {
       this.shooterIO = new ShooterIOFlex(leftMotorCanID, rightMotorCanID);
    }

    public void runVoltage(double volts)
    {
        if ((volts >= -12.0) && (volts <= 12.0))
        {
            shooterIO.setVoltage(volts);
        }
    }
    public void stop() {
        runVoltage(0.0);
    }
  
  @Override
  public void periodic() {
    shooterIO.updateInputs(shooterInputs);
    Logger.processInputs("Shooter/Motors", shooterInputs);
  }

}