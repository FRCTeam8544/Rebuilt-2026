package frc.robot.subsystems.Arm;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.Faults;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ArmIOMax implements ArmIO {

  private static final int stallLimit = 15;

  private final SparkMax armMotorController;

  private final AbsoluteEncoder armEncoder;
  private final SparkClosedLoopController closedLoop;
  private final SparkMaxConfig armMotorConfig;


  public ArmIOMax(int armCanId) {
    armMotorController = new SparkMax(armCanId, MotorType.kBrushless);

    armEncoder = armMotorController.getAbsoluteEncoder();
    closedLoop = armMotorController.getClosedLoopController();

    armMotorConfig = new SparkMaxConfig();
    armMotorConfig.idleMode(IdleMode.kBrake);
    armMotorConfig.smartCurrentLimit(stallLimit);
    armMotorConfig.voltageCompensation(12);
    armMotorConfig.softLimit.forwardSoftLimitEnabled(false);
    armMotorConfig.softLimit.forwardSoftLimit(kArmLowerLimit);
    armMotorConfig.softLimit.reverseSoftLimitEnabled(false);
    armMotorConfig.softLimit.reverseSoftLimit(kArmUpperLimit);

    armMotorConfig.encoder.positionConversionFactor(1);
    armMotorConfig.encoder.velocityConversionFactor(1);

   /* armMotorConfig
        .closedLoop
        .positionWrappingEnabled(false)
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        // Position control
        .p(0.0001, ClosedLoopSlot.kSlot0) //was 0.0008
        .i(0.00000, ClosedLoopSlot.kSlot0)
        .d(0.000000, ClosedLoopSlot.kSlot0); //0.00001
*/
    
    // armMotorConfig.closedLoop.feedForward.kS(kS);
   //  armMotorConfig.closedLoop.feedForward.kV(Constants.Neo.nominalFF);
    //                                          ClosedLoopSlot.kSlot0);

    armMotorController.configure(
        armMotorConfig,
        com.revrobotics.ResetMode.kResetSafeParameters,
        com.revrobotics.PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(ArmIOInputs inOutData) {
    inOutData.connected = true;
    inOutData.velocity = (float) armEncoder.getVelocity();
    inOutData.position = (float) armEncoder.getPosition();
    inOutData.motorTemperature = (float) armMotorController.getMotorTemperature();

    // Fault codes
    Faults armFaults = armMotorController.getFaults();
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
  
  // @Override
  public void setPosition(double rotations) {
    closedLoop.setSetpoint(rotations, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void setVoltage(double volts) {
    armMotorController.setVoltage(volts);
  }

}
