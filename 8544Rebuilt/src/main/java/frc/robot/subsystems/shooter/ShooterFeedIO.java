
package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterFeedIO {
    
  @AutoLog
  public static class ShooterFeedIOInputs {
    // Inputs
    public boolean connected = false;

    public float velocity = 0.0f;

    public float motorTemperature = 0.0f;

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

    public float velocitySetPoint = 0.0f; // Percent of max motor speed (0...1)
    public float voltageSetPoint = 0.0f; // Motor voltage, usually not directly controlled
  }

  public default void updateInputs(ShooterFeedIOInputs inOutData) {}

  public default void setVelocity(double rpm) {}
  public default void setVoltage(double volts) {}
}