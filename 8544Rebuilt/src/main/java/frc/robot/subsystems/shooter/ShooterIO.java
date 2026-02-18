package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    
  @AutoLog
  public static class ShooterIOInputs {
    // Inputs
    public boolean connected = false;

    public float motorVelocity = 0.0f;
    public float flywheelVelocity = 0.0f;
    public float leaderMotorTemperature = 0.0f;
    public float followMotorTemperature = 0.0f;

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

    public double velocitySetPoint = 0.0; // Requested flywheel output RPM
    public double voltageSetPoint= 0.0; // Motor voltage, usually not directly controlled
  }

  public default void updateInputs(ShooterIOInputs inOutData) {}
  public default void setVelocity(double rpm) {}
  public default void setVoltage(double volts) {}
}