package frc.robot.subsystems.Intake;

import java.lang.constant.Constable;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.Faults;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants;

import com.revrobotics.spark.config.SparkMaxConfig;


public class IntakeIOMax implements IntakeIO {

  private static final int kStallLimit = 30;

  private final SparkMax rollerMotorController;

  private final RelativeEncoder rollerEncoder;
  private final SparkClosedLoopController closedLoop;
  private final SparkMaxConfig rollerMotorConfig;

  public IntakeIOMax(int rollerCanId) {
    rollerMotorController = new SparkMax(rollerCanId, MotorType.kBrushless);

    rollerEncoder = rollerMotorController.getEncoder();
    closedLoop = rollerMotorController.getClosedLoopController();

    rollerMotorConfig = new SparkMaxConfig();
    rollerMotorConfig.idleMode(IdleMode.kBrake);
    rollerMotorConfig.smartCurrentLimit(kStallLimit);
    rollerMotorConfig.voltageCompensation(Constants.kNominalVoltage);
    rollerMotorConfig.inverted(true);
    rollerMotorConfig.softLimit.forwardSoftLimitEnabled(false);
    rollerMotorConfig.softLimit.reverseSoftLimitEnabled(false);
    
    rollerMotorConfig.encoder.positionConversionFactor(1);
    rollerMotorConfig.encoder.velocityConversionFactor(1);

  /* rollerMotorConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // Velocity control
          .p(0.0001, ClosedLoopSlot.kSlot0) //was 0.0008
          .i(0.00000, ClosedLoopSlot.kSlot0)
          .d(0.000000, ClosedLoopSlot.kSlot0); //0.00001
  */
    // rollerMotorConfig.closedLoop.feedForward.kS(kS);
   //  rollerMotorConfig.closedLoop.feedForward.kV(Constants.Neo.nominalFF);
    //                                          ClosedLoopSlot.kSlot0);

    rollerMotorController.configure(
        rollerMotorConfig,
        com.revrobotics.ResetMode.kResetSafeParameters,
        com.revrobotics.PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(IntakeIOInputs inOutData) {
    inOutData.connected = true;
    inOutData.velocity = (float) (rollerEncoder.getVelocity() * kMotorToOutputRatio);
    inOutData.motorTemperature = (float) rollerMotorController.getMotorTemperature();

    // Fault codes
    Faults rollerFaults = rollerMotorController.getFaults();
    inOutData.faultCan = rollerFaults.can;
    inOutData.faultTemperature = rollerFaults.temperature;
    inOutData.faultSensor = rollerFaults.sensor;
    inOutData.faultGateDriver = rollerFaults.gateDriver;
    inOutData.faultEscEeprom = rollerFaults.escEeprom;
    inOutData.faultFirmware = rollerFaults.firmware;

    // Outputs
    inOutData.busVoltage = (float) rollerMotorController.getBusVoltage();
    inOutData.outputDuty = (float) rollerMotorController.getAppliedOutput(); // -1 to 1 percent applied of bus voltage
    inOutData.outputCurrent = (float) rollerMotorController.getOutputCurrent();
    inOutData.outputVoltage = (float) (rollerMotorController.getAppliedOutput() * Constants.kNominalVoltage);
  }

  @Override
  public void setVelocity(double rpm) {
    closedLoop.setSetpoint(rpm, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void setVoltage(double volts) {
    rollerMotorController.setVoltage(volts);
  }
}
