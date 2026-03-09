package frc.robot.subsystems.Arm;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
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

  
  // Permitted voltage limits during arm extend
  private InterpolatingDoubleTreeMap extendPosToVoltLimtMap = new InterpolatingDoubleTreeMap();
  private InterpolatingDoubleTreeMap retractPosToVoltLimitMap = new InterpolatingDoubleTreeMap();

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

    buildPosToVoltMap();
  }


  // Open loop control

  // Retrict duty to mapped limits based on forward (positive) or reverse (negtive)
  public void runOpenLoopLimited(double duty) {
    double limitedDuty = getVoltageLimit(armInputs.position) / Constants.kNominalVoltage;
    if (duty > 0.0) {
      runOpenLoop( Math.min(limitedDuty, duty) );
    }
    else {
      runOpenLoop( Math.max(limitedDuty, duty) );
    }
  }

  private double getVoltageLimit(double position) {
    double voltage = 0.0;
    if (voltage > 0.0) {
      voltage = Math.min(extendPosToVoltLimtMap.get(position), voltage);
    }
    else {
      // Reverse is negative voltage so use max to limit
      voltage = Math.max(retractPosToVoltLimitMap.get(position), voltage);
    }

    return voltage;
  }

  public void runOpenLoop(double duty) {
    
    double adjustedDuty = duty;
    if (adjustedDuty > 1.0) {
      adjustedDuty = 1.0;
    }
    else if (adjustedDuty < -1.0) {
      adjustedDuty = -1.0;
    }

    armInputs.voltageLimit = (float) getVoltageLimit(armInputs.position);
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
    if (Math.abs(armInputs.velocity) > 0.0) { 
       armInputs.positionSetPoint = armInputs.position;
       armIO.setPosition(armInputs.position);
       // TODO account for velocity to alter set point to be less jerky
    }
    else {
      armInputs.positionSetPoint = armInputs.position;
      armIO.setPosition(armInputs.position);
    }
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

  
  private void buildPosToVoltMap() {
    final double maxVoltage = 0.6 * Constants.kNominalVoltage;
    final double trickleVoltage = 0.006;
    final double overshootTolerance = 0.03;
    final double fwdSlowPercent = 0.8;
    // Extension map - positive voltage
    extendPosToVoltLimtMap.put(0.0, maxVoltage);
    extendPosToVoltLimtMap.put(ArmIO.kArmReverseLimit, maxVoltage); 
    extendPosToVoltLimtMap.put(fwdSlowPercent * ArmIO.kArmForwardLimit, maxVoltage);
    extendPosToVoltLimtMap.put(ArmIO.kArmForwardLimit, trickleVoltage);
    extendPosToVoltLimtMap.put(ArmIO.kArmForwardLimit + overshootTolerance, 0.0);
    extendPosToVoltLimtMap.put(1.0,0.0);

    // Retract map - negative voltage
    final double revSlowPercent = 1.2; // Greater then one to apply before rev limit is hit
    retractPosToVoltLimitMap.put(1.0, -maxVoltage);
    retractPosToVoltLimitMap.put(ArmIO.kArmForwardLimit, -maxVoltage);
    retractPosToVoltLimitMap.put(revSlowPercent * ArmIO.kArmReverseLimit, -maxVoltage);
    retractPosToVoltLimitMap.put(ArmIO.kArmReverseLimit, -trickleVoltage);
    retractPosToVoltLimitMap.put(ArmIO.kArmReverseLimit - overshootTolerance, 0.0);
    retractPosToVoltLimitMap.put(0.0, 0.0);
  }
}