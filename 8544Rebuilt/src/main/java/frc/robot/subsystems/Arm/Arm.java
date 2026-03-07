package frc.robot.subsystems.Arm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {

  // ARM coordinates, assuming encoder is on the gearbox output
  // Clockwise motor movement will bring the arm up.
  // Counterclockwise motor movement will bring arm down.
  // Approximately a 3 to 1 gear ratio.

  private static final int armCanId = 27;
  private final ArmIO armIO;
  private final ArmIOInputsAutoLogged armInputs = new ArmIOInputsAutoLogged();

  // Provide the current arm position in rotations
  public DoubleSupplier armPositionSupplier =
    () -> {
      return armInputs.position;
    };

  // Provide the current arm position set point in rotations
  // Enable when PID control of arm works TODO
 /* public DoubleSupplier armPositionSetPointSupplier = 
    () -> {
      return armInputs.positionSetPoint;
    };
*/
  public Arm() {
    this.armIO = new ArmIOMax(armCanId);

    setupDefaultDashboard();
  }

  // Open loop control

  public void runOpenLoop(double duty) {
    
    double adjustedDuty = duty;
    if (adjustedDuty > 1.0) {
      adjustedDuty = 1.0;
    }
    else if (adjustedDuty < -1.0) {
      adjustedDuty = -1.0;
    }

    armInputs.voltageSetPoint = adjustedDuty * Constants.kNominalVoltage;
    armInputs.positionSetPoint = 0.0;

    armIO.setVoltage(armInputs.voltageSetPoint);
  }

  public void stopOpenLoop() {
    runOpenLoop(0);
  }

  // Closed loop control

  public void runToPosition(double rotations) {
    armInputs.positionSetPoint = rotations;
    armInputs.voltageSetPoint = 0.0;
    armIO.setPosition(armInputs.positionSetPoint);
  } 

  public void holdPosition() {
    armInputs.positionSetPoint = armInputs.position;
    armIO.setPosition(armInputs.position);
  }

  public void stopMotors() {
    armIO.setVoltage(0.0);
  }

  @Override
  public void periodic() {
    armIO.updateInputs(armInputs);
    Logger.processInputs("Arm",armInputs);

    
    SmartDashboard.putNumber("Arm Position Setpoint", armInputs.positionSetPoint);
    SmartDashboard.putNumber("Arm Volts Setpoint", armInputs.voltageSetPoint);
    SmartDashboard.putNumber("Arm Motor Temp", armInputs.motorTemperature);
    
    // Controls
    SmartDashboard.putBoolean("Arm Brake Enabled", armInputs.motorBrakeEnabled);
  }
 
  
  private void setupDefaultDashboard()
  {
    SmartDashboard.setDefaultNumber("Arm Position Setpoint", armInputs.positionSetPoint);
    SmartDashboard.setDefaultNumber("Arm Volts Setpoint", armInputs.voltageSetPoint);
    SmartDashboard.setDefaultNumber("Arm Motor Temp", armInputs.motorTemperature);
  }
}