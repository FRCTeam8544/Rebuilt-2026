package frc.robot.subsystems.shooter;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.Faults;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkClosedLoopController;

import frc.robot.Constants;

import com.revrobotics.RelativeEncoder;

public class ShooterIOFlex implements ShooterIO {
  
    private static final int stallLimit = 40;

    private final SparkFlex leaderMotorController;
    private final SparkFlex followMotorController;

    private final RelativeEncoder leaderEncoder;
    private final SparkClosedLoopController closedLoop;
    private final SparkFlexConfig leaderMotorConfig;
    private final SparkFlexConfig followMotorConfig;

  public ShooterIOFlex(int leaderCanId, int followCanId) {
    leaderMotorController = new SparkFlex(leaderCanId, MotorType.kBrushless);
    followMotorController = new SparkFlex(followCanId, MotorType.kBrushless);

    leaderEncoder = leaderMotorController.getEncoder();
    closedLoop = leaderMotorController.getClosedLoopController();

    leaderMotorConfig = new SparkFlexConfig();
    followMotorConfig = new SparkFlexConfig();

    leaderMotorConfig.idleMode(IdleMode.kBrake);
    leaderMotorConfig.smartCurrentLimit(stallLimit);
    leaderMotorConfig.voltageCompensation(12);
    leaderMotorConfig.softLimit.forwardSoftLimitEnabled(false);
    leaderMotorConfig.softLimit.reverseSoftLimitEnabled(false);
    leaderMotorConfig.closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // Velocity control
          .p(0.00003, ClosedLoopSlot.kSlot0)
          .i(0.00000, ClosedLoopSlot.kSlot0)
          .d(0.00001, ClosedLoopSlot.kSlot0);
    leaderMotorConfig.closedLoop.feedForward.kV(Constants.NeoVortex.nominalFF, 
                                                ClosedLoopSlot.kSlot0);
    
    leaderMotorController.configure(leaderMotorConfig, 
                              com.revrobotics.ResetMode.kResetSafeParameters,
                              com.revrobotics.PersistMode.kPersistParameters);
    
    followMotorConfig.idleMode(IdleMode.kBrake);
    followMotorConfig.smartCurrentLimit(stallLimit);
    followMotorConfig.follow(leaderCanId, true); 
    followMotorConfig.voltageCompensation(12);
    followMotorConfig.softLimit.forwardSoftLimitEnabled(false);
    followMotorConfig.softLimit.reverseSoftLimitEnabled(false);
    
     followMotorController.configure(followMotorConfig, 
                              com.revrobotics.ResetMode.kResetSafeParameters,
                              com.revrobotics.PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(ShooterIOInputs inOutData)
  {
    inOutData.connected = true;
    inOutData.velocity = (float) leaderEncoder.getVelocity();
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
  }

  @Override
  public void setVelocity(double rpm) {
    closedLoop.setSetpoint(rpm, ControlType.kVelocity,ClosedLoopSlot.kSlot0);
  }

  @Override
  public void setVoltage(double volts) {
    leaderMotorController.setVoltage(volts);
  }

}