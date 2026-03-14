package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {

  @AutoLog
  public static class ClimberIOInputs {
    // Controls
    public boolean motorBrakeEnabled = true;

    // Inputs
    public boolean connected = false;

    public float velocity = 0.0f;
    public float position = 0.0f;

    public float motorTemperature = 0.0f;

    // Fault codes
    public boolean faultSensor;
    public boolean faultCan;
    public boolean faultTemperature;
    public boolean faultGateDriver;
    public boolean faultEscEeprom;
    public boolean faultFirmware;

    // Outputs
    public double feedForward = 0.0;
    public float busVoltage = 0;
    public float outputDuty = 0; // -1 to 1 percent applied of bus voltage
    public float outputCurrent = 0;
    public float outputVoltage = 0;

    public double positionSetPoint = 0.0;
    public double voltageSetPoint = 0.0; // Motor voltage, usually not directly controlled
  }

  public default void updateInputs(ClimberIOInputs inOutData) {}

  public default void setPosition(double rotations) {}

  public default void setVoltage(double volts) {}

  public default void setBrakeMode(boolean enable) {}
}
