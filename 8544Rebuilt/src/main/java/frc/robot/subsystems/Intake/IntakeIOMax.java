package frc.robot.subsystems.Intake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.Faults;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.subsystems.Intake.*;

public class IntakeIOMax implements IntakeIO {

  private static final int stallLimit = 20;

public static double realPosition;
 
  private final SparkMax armMotorController;

  private final AbsoluteEncoder armEncoder;
  private final SparkClosedLoopController closedLoop;
  private final SparkMaxConfig armMotorConfig;
  private static final double gearRatio = 1/100.0;

  public IntakeIOMax(int armCanId) {
    armMotorController = new SparkMax(armCanId, MotorType.kBrushless);

    armEncoder = armMotorController.getAbsoluteEncoder();
    closedLoop = armMotorController.getClosedLoopController();

    armMotorConfig = new SparkMaxConfig();
    armMotorConfig.idleMode(IdleMode.kBrake);
    armMotorConfig.smartCurrentLimit(stallLimit);
    armMotorConfig.voltageCompensation(12);
    armMotorConfig.softLimit.forwardSoftLimitEnabled(true);
    armMotorConfig.softLimit.reverseSoftLimitEnabled(true);
    armMotorConfig.softLimit.forwardSoftLimit(0.2);
    armMotorConfig.softLimit.reverseSoftLimit(.8);
  //  armMotorConfig.absoluteEncoder.positionConversionFactor(1/100.0);
   // armMotorConfig.absoluteEncoder.velocityConversionFactor(1/100.0);
    armMotorConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        // Velocity control
        .p(0.1000, ClosedLoopSlot.kSlot0)
        .i(0.00000, ClosedLoopSlot.kSlot0)
        .d(0.00002, ClosedLoopSlot.kSlot0);
    // armMotorConfig.closedLoop.feedForward.kS(kS);
    // armMotorConfig.closedLoop.feedForward.kV(Constants.NeoVortex.nominalFF,
    //                                          ClosedLoopSlot.kSlot0);

    armMotorController.configure(
        armMotorConfig,
        com.revrobotics.ResetMode.kResetSafeParameters,
        com.revrobotics.PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(IntakeIOInputs inOutData) {
    inOutData.connected = true;
    inOutData.velocity = (float) armEncoder.getVelocity();
    inOutData.position = (float) armEncoder.getPosition();
    inOutData.armMotorTemperature = (float) armMotorController.getMotorTemperature();

    // Fault codes
    // inOutData.feedForward = feedForward;
    Faults armFaults = armMotorController.getFaults();
    Faults followFaults = armMotorController.getFaults();
    inOutData.faultCan = armFaults.can;
    inOutData.faultTemperature = armFaults.temperature;
    inOutData.faultSensor = armFaults.sensor;
    inOutData.faultGateDriver = armFaults.gateDriver;
    inOutData.faultEscEeprom = armFaults.escEeprom;
    inOutData.faultFirmware = armFaults.firmware;

    // Outputs
    inOutData.busVoltage = (float) armMotorController.getBusVoltage();
    inOutData.outputDuty =
        (float) armMotorController.getAppliedOutput(); // -1 to 1 percent applied of bus voltage
    inOutData.outputCurrent = (float) armMotorController.getOutputCurrent();
    inOutData.outputVoltage = (float) armMotorController.getAppliedOutput() * 12.0f;
  }
  /*
    @Override
    public void setFeedForward(double ff) {
  armMotorConfig.closedLoop.feedForward.kV(feedForward,
                                                  ClosedLoopSlot.kSlot0);
   */
  // armMotorController.configure(armMotorConfig,
  //                         com.revrobotics.ResetMode.kNoResetSafeParameters,
  //                       com.revrobotics.PersistMode.kNoPersistParameters);

  // @Override
  public void setPosition(double rotations) {
    closedLoop.setSetpoint(rotations, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }
  
  public double realPosition() {
    return armEncoder.getPosition();
  }

  @Override
  public void setVoltage(double volts) {
    armMotorController.setVoltage(volts);
  }
}
