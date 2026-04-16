package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.Constants;


public interface IntakeIO {

  public static final double kMotorToOutputRatio = 1.0/4.0; // 4 to 1 gearbox
  public static final double kOutputToMotorRatio = 1.0 / kMotorToOutputRatio;

  public static final double kMaxIntakeRPM = Constants.Neo.freeSpeedRPM * kMotorToOutputRatio;
  @AutoLog
  public static class IntakeIOInputs {
    // Inputs
    public boolean connected = false;

    public float velocity = 0.0f;
    public float position = 0.0f;
    public float motorTemperature = 0.0f;

    public double velocitySetPoint = 0;

    // Fault codes
    public boolean faultSensor;
    public boolean faultCan;
    public boolean faultTemperature;
    public boolean faultGateDriver;
    public boolean faultEscEeprom;
    public boolean faultFirmware;

    // Outputs
    public float busVoltage = 0;
    public float outputDuty = 0; // -1 to 1 percent applied of bus voltage
    public float outputCurrent = 0;
    public float outputVoltage = 0;

    public double positionSetPoint = 0.0; // Position in rotations
    public double voltageSetPoint = 0.0; // Motor voltage, usually not directly controlled

    // Game piece state (populated by sim, default 0/false for real hardware)
    public int fuelCount = 0;
    public boolean hasFuel = false;
  }

  public default void updateInputs(IntakeIOInputs inOutData) {}

  public default void setVoltage(double volts) {}
  public default void setVelocity(double rpm) {}
}
