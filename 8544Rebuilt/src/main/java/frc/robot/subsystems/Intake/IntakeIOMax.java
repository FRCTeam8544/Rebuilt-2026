package frc.robot.subsystems.Intake;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.Faults;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkClosedLoopController;

import frc.robot.Constants;

import com.revrobotics.RelativeEncoder;

public class IntakeIOMax implements IntakeIO {
  
    private static final int stallLimit = 60;
    
    private static final double kS = 0.14;
    private double feedForward = 0.0;

    private final SparkMax armMotorController;

    private final RelativeEncoder armEncoder;
    private final SparkClosedLoopController closedLoop;
    private final SparkMaxConfig armMotorConfig;

  public IntakeIOMax(int armCanId, int followCanId) {
    armMotorController = new SparkMax(armCanId, MotorType.kBrushless);


    armEncoder = armMotorController.getEncoder();
    closedLoop = armMotorController.getClosedLoopController();

    armMotorConfig = new SparkMaxConfig();
    armMotorConfig.idleMode(IdleMode.kCoast);
    armMotorConfig.smartCurrentLimit(stallLimit);
    armMotorConfig.voltageCompensation(12);
    armMotorConfig.softLimit.forwardSoftLimitEnabled(false);
    armMotorConfig.softLimit.reverseSoftLimitEnabled(false);
    armMotorConfig.closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // Velocity control
          .p(0.0000, ClosedLoopSlot.kSlot0)
          .i(0.00000, ClosedLoopSlot.kSlot0)
          .d(0.00000, ClosedLoopSlot.kSlot0);
   // armMotorConfig.closedLoop.feedForward.kS(kS);
    //armMotorConfig.closedLoop.feedForward.kV(Constants.NeoVortex.nominalFF, 
      //                                          ClosedLoopSlot.kSlot0);
    
    armMotorController.configure(armMotorConfig, 
                              com.revrobotics.ResetMode.kResetSafeParameters,
                              com.revrobotics.PersistMode.kPersistParameters);
    
  }

  @Override
  public void updateInputs(IntakeIOInputs inOutData)
  {
    inOutData.connected = true;
    inOutData.velocity = (float) armEncoder.getVelocity();
    inOutData.position = (float) armEncoder.getPosition();
    inOutData.armMotorTemperature = (float) armMotorController.getMotorTemperature();

    // Fault codes
    inOutData.feedForward = feedForward;
    Faults armFaults = armMotorController.getFaults();
    Faults followFaults = armMotorController.getFaults();
    inOutData.faultCan = armFaults.can || followFaults.can;
    inOutData.faultTemperature = armFaults.temperature || followFaults.temperature;
    inOutData.faultSensor = armFaults.sensor || followFaults.sensor;
    inOutData.faultGateDriver = armFaults.gateDriver || followFaults.gateDriver;
    inOutData.faultEscEeprom = armFaults.escEeprom || followFaults.escEeprom;
    inOutData.faultFirmware = armFaults.firmware || followFaults.firmware;

    // Outputs
    inOutData.busVoltage = (float) armMotorController.getBusVoltage();
    inOutData.outputDuty = (float) armMotorController.getAppliedOutput(); // -1 to 1 percent applied of bus voltage
    inOutData.outputCurrent = (float) armMotorController.getOutputCurrent();
    inOutData.outputVoltage = (float) armMotorController.getAppliedOutput() * 12.0f;
  }

  @Override
  public void setFeedForward(double ff) {
armMotorConfig.closedLoop.feedForward.kV(feedForward, 
                                                ClosedLoopSlot.kSlot0);

   // armMotorController.configure(armMotorConfig, 
     //                         com.revrobotics.ResetMode.kNoResetSafeParameters,
       //                       com.revrobotics.PersistMode.kNoPersistParameters);
    
  }

  //@Override
  public void setPosition(double rotations, double feedForward) {
    closedLoop.setSetpoint(rotations, ControlType.kPosition,ClosedLoopSlot.kSlot0, feedForward);
  }

  @Override
  public void setVoltage(double volts) {
    armMotorController.setVoltage(volts);
  }

}
