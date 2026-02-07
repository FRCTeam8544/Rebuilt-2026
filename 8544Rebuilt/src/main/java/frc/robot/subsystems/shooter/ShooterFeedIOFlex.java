package frc.robot.subsystems.shooter;

import com.revrobotics.spark.SparkBase.Faults;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import frc.robot.subsystems.shooter.ShooterIO;

public class ShooterFeedIOFlex implements ShooterFeedIO {
  
    private static final int stallLimit = 40;

    private final SparkFlex motorController;

    //private static RelativeEncoder encoder = leaderMotorController.getExternalEncoder();
    //private static SparkClosedLoopController closedLoop = leftMotorController.getClosedLoopController();
    private final SparkFlexConfig motorConfig;

  public ShooterFeedIOFlex(int canId) {
    motorController = new SparkFlex(canId, MotorType.kBrushless);

    motorConfig = new SparkFlexConfig();

    motorConfig.idleMode(IdleMode.kBrake);
    motorConfig.smartCurrentLimit(stallLimit);
    motorConfig.voltageCompensation(12);
    motorConfig.softLimit.forwardSoftLimitEnabled(false);
    motorConfig.softLimit.reverseSoftLimitEnabled(false);

    motorController.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(ShooterFeedIOInputs inOutData)
  {
    inOutData.connected = true;
    inOutData.velocity = 0;// TODO need encoder;
    inOutData.motorTemperature = (float) motorController.getMotorTemperature();

    // Fault codes
    Faults faults = motorController.getFaults();
    inOutData.faultCan = faults.can;
    inOutData.faultTemperature = faults.temperature;
    inOutData.faultSensor = faults.sensor;
    inOutData.faultGateDriver = faults.gateDriver;
    inOutData.faultEscEeprom = faults.escEeprom;
    inOutData.faultFirmware = faults.firmware;

    // Outputs
    inOutData.busVoltage = (float) motorController.getBusVoltage();
    inOutData.outputDuty = (float) motorController.getAppliedOutput(); // -1 to 1 percent applied of bus voltage
    inOutData.outputCurrent = (float) motorController.getOutputCurrent();

    inOutData.velocitySetPoint = 0.0f; // Percent of max motor speed (0...1)
    inOutData.voltageSetPoint = 0.0f; // Motor voltage, usually not directly controlled
  }

  @Override
  public void setVelocity(double rpm) {
    // TODO, using closed loop controller
  }

  @Override
  public void setVoltage(double volts) {
    motorController.setVoltage(volts);
  }

}