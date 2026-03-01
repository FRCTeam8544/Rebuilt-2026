package frc.robot.subsystems.climber;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class Climber extends SubsystemBase {

  private static final int climberCanId = 29;
 //private static final int climberCanCoderId = 84;??
  
  private final ClimberIO climberIO;
  private final ClimberIOInputsAutoLogged climberInputs = new ClimberIOInputsAutoLogged();

  private final double minPositionLimit = 0;   // Rotations
  private final double maxPositionLimit = 0.5; // Rotations

  
  public DoubleSupplier outputVoltageSupplier = 
    () -> {
      return climberInputs.outputVoltage;
    };

  // Zero faces to the just below front of robot
  public DoubleSupplier positionSupplier =
    () -> {
      // TODO provide in degrees, use rotations for now
      //return Units.rotationsToDegrees(climberInputs.absolutePosition);
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
    this.climberIO = new ClimberIOFlex(climberCanId);
    
    setupDefaultDashboard();
  }

  public void runArmToPosition(double rotations) {
      if (rotations > maxPositionLimit) {
        rotations = 1.0;
      }
      else if (rotations < minPositionLimit) {
        rotations = 0.0;
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

// --- ADDED FOR LED CONTROL DURING CLIMBING ---
  // Climber position (in rotations) above which the arm is considered engaged for climbing
  private static final double CLIMB_ENGAGED_POSITION_ROTATIONS = 0.15;
  // Minimum voltage magnitude to be considered actively climbing
  private static final double CLIMBING_MIN_VOLTAGE_VOLTS = 1.0;

  /**
   * Returns true when the climber arm has been extended past the engagement threshold,
   * indicating the robot is in or near a climbing position.
   */
  public boolean isAtClimbPosition() {
    return climberInputs.position > CLIMB_ENGAGED_POSITION_ROTATIONS;
  }

  /**
   * Returns true when the climber is being actively driven with open-loop voltage.
   */
  public boolean isClimbing() {
    return Math.abs(climberInputs.voltageSetPoint) > CLIMBING_MIN_VOLTAGE_VOLTS;
  }
// --- END  ---

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
  }


private void setupDefaultDashboard()
{
  SmartDashboard.setDefaultNumber("Climber Position", climberInputs.position);
  SmartDashboard.setDefaultNumber("Climber SetPoint", climberInputs.positionSetPoint);
  SmartDashboard.setDefaultNumber("Climber Motor Temp", climberInputs.motorTemperature);
}

}