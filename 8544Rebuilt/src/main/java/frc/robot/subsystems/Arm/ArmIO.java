package frc.robot.subsystems.Arm;


import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  
  public static final double kArmUpperLimit = 0.8;
  public static final double kArmLowerLimit = 0.1;

  public static final double kMotorToOutputRatio = 1.0/100.0; // 100 to 1 gearbox
  public static final double kOutputToMotorRatio = 1.0 / kMotorToOutputRatio;

  @AutoLog
  public static class ArmIOInputs {
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
    public float busVoltage = 0;
    public float outputDuty = 0; // -1 to 1 percent applied of bus voltage
    public float outputCurrent = 0;
    public float outputVoltage = 0;

    public double voltageSetPoint = 0.0; // Motor voltage, usually not directly controlled
    public double positionSetPoint = 0;
  }

  public default void updateInputs(ArmIOInputs inOutData) {}

  public default void setVoltage(double volts) {}
  public default void setPosition(double rotations){}
}