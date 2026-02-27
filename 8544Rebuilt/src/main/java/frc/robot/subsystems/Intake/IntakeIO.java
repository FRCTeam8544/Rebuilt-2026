package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;


public interface IntakeIO {

  @AutoLog
  public static class IntakeIOInputs {
    // Inputs
    public boolean connected = false;

    public float velocity = 0.0f;
    public float position = 0.0f;
    public float armMotorTemperature = 0.0f;

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
  }

  public default void updateInputs(IntakeIOInputs inOutData) {}
  public default void setPosition(double rotations) {}

  public default void setVoltage(double volts) {}
}
