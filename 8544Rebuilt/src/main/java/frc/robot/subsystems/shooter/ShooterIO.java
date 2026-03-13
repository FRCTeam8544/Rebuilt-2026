package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    
  @AutoLog
  public static class ShooterIOInputs {
    // Inputs
    public boolean connected = false;

    public double motorVelocity = 0.0f;
    public double flywheelVelocity = 0.0f;
    public float leaderMotorTemperature = 0.0f;
    public float followMotorTemperature = 0.0f;

    // Fault codes
    boolean faultSupplyUnderVoltage = false;
    boolean faultBridgeBrownout = false;
    boolean faultTemperature = false;
    boolean faultControllerTemperature = false;
    boolean faultHardwareLeaderMotor = false;
    boolean faultHardwareFollowMotor = false;
    boolean faultStatorCurrentLimitLeaderMotor = false;
    boolean faultStatorCurrentLimitFollowMotor = false;

    // Dashboard Controls
    public boolean flywheelAutoRPMToggle = true;

    // Outputs
    public boolean maxFlywheelSpeedHit = false;
    public double feedForward = 0.0;
    public float busVoltage = 0;
    public float outputDuty = 0; // -1 to 1 percent applied of bus voltage
    public float outputCurrent = 0;
    public float outputVoltage = 0;
    public double outputPower = 0;
    public double flywheelVelocitySetPoint = 0.0; // Requested flywheel output RPM
    public double voltageSetPoint= 0.0; // Motor voltage, usually not directly controlled
  }

  public default void updateInputs(ShooterIOInputs inOutData) {}
  public default void setVelocity(double rpm) {}
  public default void setVoltage(double volts) {}
}