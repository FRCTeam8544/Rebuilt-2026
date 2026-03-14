package frc.robot.subsystems.climber;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {

  private static final int climberCanId = 29;
 private static final int encoderCanId = 31;
  
  private final ClimberIO climberIO;
  private final ClimberIOInputsAutoLogged climberInputs = new ClimberIOInputsAutoLogged();

  private final double minPositionLimit = 0;   // Rotations
  private final double maxPositionLimit = 2.5; // Rotations TODO!!!


  // Zero faces to the front of robot
  // Positive towards the rear of the robot
  public DoubleSupplier positionSupplier =
    () -> {
      return climberInputs.position;
    };
  
  public DoubleSupplier positionSetPointSupplier =
    () -> {
      // TODO provide in degrees, use rotations for now
      //return Units.rotationsToDegrees(climberInputs.positionSetPoint);
      return climberInputs.positionSetPoint;
    };

  public DoubleSupplier motorTempSupplier =
    () -> {
      return climberInputs.motorTemperature;
    };

  public Climber() {
    this.climberIO = new ClimberIOFlex(climberCanId,encoderCanId);
    
    setupDefaultDashboard();
  }

  public void enableCoastMode() {
    setBrakeMode(false);
  }

  public void enableBrakeMode() {
    setBrakeMode(true);
  }

  private void setBrakeMode(boolean enable) {
    // Update the dashboard as if the user did it.
    // The change will be noticed in the next periodic update and the motor config changed
    SmartDashboard.putBoolean("Climber Brake Enabled", false);
  }

  public void runArmToPosition(double rotations) {
      if (rotations > maxPositionLimit) {
        rotations = maxPositionLimit;
      }
      else if (rotations < minPositionLimit) {
        rotations = minPositionLimit;
      }
      climberInputs.voltageSetPoint = 0.0;
      climberInputs.positionSetPoint = (float) rotations;

      climberIO.setPosition(climberInputs.position);
  }

  public void holdArmPosition() {
    climberInputs.voltageSetPoint = 0.0;
    climberInputs.positionSetPoint = climberInputs.position;
    climberIO.setPosition(climberInputs.positionSetPoint);
  }

  public void runArmOpenLoop(double duty) {
    double adjustedDuty = duty;
    if (adjustedDuty > 1.0)
    {
      adjustedDuty = 1.0;
    }
    else if (adjustedDuty < -1.0) {
      adjustedDuty = -1.0;
    }

    climberInputs.positionSetPoint = 0.0;
    climberInputs.voltageSetPoint = (float) adjustedDuty * Constants.kNominalVoltage;
    climberIO.setVoltage(climberInputs.voltageSetPoint);
  }


  @Override
  public void periodic() {
    climberIO.updateInputs(climberInputs);
    Logger.processInputs("Climber", climberInputs);

    SmartDashboard.putNumber("Climber Position", climberInputs.position);
    SmartDashboard.putNumber("Climber Setpoint", climberInputs.positionSetPoint);
    SmartDashboard.putNumber("Climber Motor Temp", climberInputs.motorTemperature);
    SmartDashboard.putNumber("Climber Encoder Position", climberInputs.encoderPosition);

    
    boolean oldBrakeMode = climberInputs.motorBrakeEnabled;
    climberInputs.motorBrakeEnabled = 
      SmartDashboard.getBoolean("Climber Brake Enabled", climberInputs.motorBrakeEnabled);
    if (oldBrakeMode != climberInputs.motorBrakeEnabled) {
      climberIO.setBrakeMode(climberInputs.motorBrakeEnabled);
    }
  }


private void setupDefaultDashboard()
{
  SmartDashboard.setDefaultNumber("Climber Position", climberInputs.position);
  SmartDashboard.setDefaultNumber("Climber SetPoint", climberInputs.positionSetPoint);
  SmartDashboard.setDefaultNumber("Climber Motor Temp", climberInputs.motorTemperature);
  SmartDashboard.setDefaultNumber("Climber Encoder Position", climberInputs.encoderPosition);
}

}