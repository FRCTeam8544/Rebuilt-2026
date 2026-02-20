package frc.robot.subsystems.shooter;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.Faults;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.RelativeEncoder;

public class ShooterIOFlex implements ShooterIO {
  
    private static final int stallLimit = 60;
    
    private static final double kMeasuredKv = 590.0;
    private static final double kS = 0.1435; // Static mechanism friction

    private double currentFeedForward = 0.0;

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

    leaderMotorConfig.idleMode(IdleMode.kCoast);
    leaderMotorConfig.smartCurrentLimit(stallLimit);
    leaderMotorConfig.voltageCompensation(12);
    leaderMotorConfig.softLimit.forwardSoftLimitEnabled(false);
    leaderMotorConfig.softLimit.reverseSoftLimitEnabled(false);
    leaderMotorConfig.closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // Velocity control
          .p(0.00015, ClosedLoopSlot.kSlot0) //was 0.00004 then 0.00020
          .i(0.00000, ClosedLoopSlot.kSlot0)
          .d(0.00002, ClosedLoopSlot.kSlot0); //was 0.000001  then 0.000009
   // leaderMotorConfig.closedLoop.feedForward.kS(kS);
    //leaderMotorConfig.closedLoop.feedForward.kV(3.605 / 10000, 
     //                                           ClosedLoopSlot.kSlot0);
    
    leaderMotorController.configure(leaderMotorConfig, 
                              com.revrobotics.ResetMode.kResetSafeParameters,
                              com.revrobotics.PersistMode.kPersistParameters);
    
    followMotorConfig.idleMode(IdleMode.kCoast);
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
    inOutData.motorVelocity = leaderEncoder.getVelocity();
    inOutData.flywheelVelocity = inOutData.motorVelocity;
    inOutData.leaderMotorTemperature = (float) leaderMotorController.getMotorTemperature();
    inOutData.followMotorTemperature = (float) followMotorController.getMotorTemperature();

    // Fault codes
    /*Faults leaderFaults = leaderMotorController.getFaults();
    Faults followFaults = leaderMotorController.getFaults();
    inOutData.faultCan = leaderFaults.can || followFaults.can;
    inOutData.faultTemperature = leaderFaults.temperature || followFaults.temperature;
    inOutData.faultSensor = leaderFaults.sensor || followFaults.sensor;
    inOutData.faultGateDriver = leaderFaults.gateDriver || followFaults.gateDriver;
    inOutData.faultEscEeprom = leaderFaults.escEeprom || followFaults.escEeprom;
    inOutData.faultFirmware = leaderFaults.firmware || followFaults.firmware;
*/
    // Outputs
    inOutData.busVoltage = (float) leaderMotorController.getBusVoltage();
    inOutData.outputDuty = (float) leaderMotorController.getAppliedOutput(); // -1 to 1 percent applied of bus voltage
    inOutData.outputCurrent = (float) leaderMotorController.getOutputCurrent();
    inOutData.outputVoltage = (float) leaderMotorController.getAppliedOutput() * 12.0f;

    inOutData.feedForward = currentFeedForward;
  }

  @Override
  public void setVelocity(double rpm) {
    final double flywheelFeedForward = 1.0 / kMeasuredKv; // Measured kV 590 of flywheel
    final double scaledFeedForward = flywheelFeedForward * rpm + kS;
    currentFeedForward = scaledFeedForward;
    closedLoop.setSetpoint(rpm, ControlType.kVelocity,ClosedLoopSlot.kSlot0, currentFeedForward);
  }

  @Override
  public void setVoltage(double volts) {
    leaderMotorController.setVoltage(volts);
  }

}