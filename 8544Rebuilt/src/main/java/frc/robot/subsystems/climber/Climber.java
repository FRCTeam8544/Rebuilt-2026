package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {

  public static final int climberCanId = 29;

  
    private final ClimberIO climberIO;
    private final ClimberIOInputsAutoLogged climberInputs = new ClimberIOInputsAutoLogged();

   // private double tuneVoltage = 0.0;
 //   private final double tuneVoltStep = 1.0 / 50.0; // 1 volt per second

    private final double minPositionLimit = 0;   // Rotations
    private final double maxPositionLimit = 0.5; // Rotations
 //   private final double armVoltageStep = 1.0 / 50.0; // 1v/s

  public Climber() {
    this.climberIO = new ClimberIOFlex(climberCanId);
  }

  public void runArmToPosition(double rotations) {
      if (rotations > 1.0) {
        rotations = 1.0;
      }
      else if (rotations < 0.0) {
        rotations = 0.0;
      }
      climberInputs.voltageSetPoint = 0.0;
      climberInputs.positionSetPoint = (float) rotations;

      climberIO.setPosition(climberInputs.position * 100.0);
  }


  public void holdArmPosition() {

      climberInputs.positionSetPoint = climberInputs.position;

      climberIO.setPosition(climberInputs.positionSetPoint);
  
  }
/* 
  public void runArmOpenLoop(double voltage) {
    double adjustedDuty = voltage + tuneVoltage;
    if (adjustedDuty > 12.0)
    {
      adjustedDuty = 12.0;
    }
    else if (voltage < 0) {
      adjustedDuty = 0.0;
    }

    climberInputs.positionSetPoint = 0.0;
    climberInputs.voltageSetPoint = (float) adjustedDuty;
    climberIO.setVoltage(climberInputs.voltageSetPoint);
  }



    public void increaseArmVoltage() {
    tuneVoltage += armVoltageStep;
    if (tuneVoltage > 12.0)
    {
      tuneVoltage = 12.0;
    }
    else if (tuneVoltage < 0) {
      tuneVoltage = 0.0;
    }
  }

   
    public void decreaseArmVoltage() {
    tuneVoltage -= armVoltageStep;
    if (tuneVoltage > 12.0)
    {
      tuneVoltage = 12.0;
    }
    else if (tuneVoltage < 0) {
      tuneVoltage = 0.0;
    }
  }
    */


  @Override
  public void periodic() {
    climberIO.updateInputs(climberInputs);
    Logger.processInputs("Climber/Motor", climberInputs);

    SmartDashboard.putNumber("Climber Position", climberInputs.position);
    SmartDashboard.putNumber("Climber Setpoint", climberInputs.positionSetPoint);
    
    SmartDashboard.putNumber("Intake Arm Temp", climberInputs.motorTemperature);
  }
}
