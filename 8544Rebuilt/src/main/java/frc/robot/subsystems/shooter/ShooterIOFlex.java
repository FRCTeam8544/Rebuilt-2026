package frc.robot.subsystems.shooter;

import com.revrobotics.spark.SparkBase.Faults;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import frc.robot.subsystems.shooter.ShooterIO;

public class ShooterIOFlex implements ShooterIO {
  
    private static final int stallLimit = 40;

    private final SparkFlex leaderMotorController;
    private final SparkFlex followMotorController;

    //private static RelativeEncoder encoder = leaderMotorController.getExternalEncoder();
    //private static SparkClosedLoopController closedLoop = leftMotorController.getClosedLoopController();
    private final SparkFlexConfig leaderMotorConfig;
    private final SparkFlexConfig followMotorConfig;

  public ShooterIOFlex(int leaderCanId, int followCanId) {
    leaderMotorController = new SparkFlex(leaderCanId, MotorType.kBrushless);
    followMotorController = new SparkFlex(leaderCanId, MotorType.kBrushless);

    leaderMotorConfig = new SparkFlexConfig();
    followMotorConfig = new SparkFlexConfig();

    leaderMotorConfig.idleMode(IdleMode.kBrake);
    leaderMotorConfig.smartCurrentLimit(stallLimit);
    leaderMotorConfig.voltageCompensation(12);
    leaderMotorConfig.softLimit.forwardSoftLimitEnabled(false);
    leaderMotorConfig.softLimit.reverseSoftLimitEnabled(false);

    leaderMotorController.configure(leaderMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    followMotorConfig.idleMode(IdleMode.kBrake);
    followMotorConfig.smartCurrentLimit(stallLimit);
    followMotorConfig.follow(leaderCanId, true); 
    followMotorConfig.voltageCompensation(12);
    followMotorConfig.softLimit.forwardSoftLimitEnabled(false);
    followMotorConfig.softLimit.reverseSoftLimitEnabled(false);
    
    followMotorController.configure(followMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(ShooterIOInputs inOutData)
  {
    inOutData.connected = true;
    inOutData.velocity = 0;// TODO need encoder;
    inOutData.leaderMotorTemperature = (float) leaderMotorController.getMotorTemperature();
    inOutData.followMotorTemperature = (float) followMotorController.getMotorTemperature();

    // Fault codes
    Faults leaderFaults = leaderMotorController.getFaults();
    Faults followFaults = leaderMotorController.getFaults();
    inOutData.faultCan = leaderFaults.can || followFaults.can;
    inOutData.faultTemperature = leaderFaults.temperature || followFaults.temperature;
    inOutData.faultSensor = leaderFaults.sensor || followFaults.sensor;
    inOutData.faultGateDriver = leaderFaults.gateDriver || followFaults.gateDriver;
    inOutData.faultEscEeprom = leaderFaults.escEeprom || followFaults.escEeprom;
    inOutData.faultFirmware = leaderFaults.firmware || followFaults.firmware;

    // Outputs
    inOutData.busVoltage = (float) leaderMotorController.getBusVoltage();
    inOutData.outputDuty = (float) leaderMotorController.getAppliedOutput(); // -1 to 1 percent applied of bus voltage
    inOutData.outputCurrent = (float) leaderMotorController.getOutputCurrent();

    inOutData.velocitySetPoint = 0.0f; // Percent of max motor speed (0...1)
    inOutData.voltageSetPoint = 0.0f; // Motor voltage, usually not directly controlled
  }

  @Override
  public void setVelocity(double rpm) {
    // TODO, using closed loop controller
  }

  @Override
  public void setVoltage(double volts) {
    leaderMotorController.setVoltage(volts);
  }

}