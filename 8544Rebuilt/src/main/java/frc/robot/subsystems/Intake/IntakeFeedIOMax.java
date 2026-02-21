package frc.robot.subsystems.Intake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.Faults;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;

public class IntakeFeedIOMax implements IntakeFeedIO {

  private static final int kFeedMaxRpm = 5676;
  private static final int stallLimit = 30;

  private final SparkMax rollerMotorController;

  private final RelativeEncoder rollerEncoder;
  private final SparkClosedLoopController closedLoop;
  private final SparkMaxConfig rollerMotorConfig;

  public IntakeFeedIOMax(int rollerCanId) {
    rollerMotorController = new SparkMax(rollerCanId, MotorType.kBrushless);

    rollerEncoder = rollerMotorController.getEncoder();
    closedLoop = rollerMotorController.getClosedLoopController();

    rollerMotorConfig = new SparkMaxConfig();
    rollerMotorConfig.idleMode(IdleMode.kBrake);
    rollerMotorConfig.smartCurrentLimit(stallLimit);
    rollerMotorConfig.voltageCompensation(12);
    rollerMotorConfig.inverted(true);
    rollerMotorConfig.softLimit.forwardSoftLimitEnabled(false);
    rollerMotorConfig.softLimit.reverseSoftLimitEnabled(false);
   /* rollerMotorConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Velocity control
        .p(0.0001, ClosedLoopSlot.kSlot0) //was 0.0008
        .i(0.00000, ClosedLoopSlot.kSlot0)
        .d(0.000000, ClosedLoopSlot.kSlot0); //0.00001
*/
       rollerMotorConfig.encoder.positionConversionFactor(1);
       rollerMotorConfig.encoder.velocityConversionFactor(1);
    // armMotorConfig.closedLoop.feedForward.kS(kS);
   //  rollerMotorConfig.closedLoop.feedForward.kV(Constants.Neo.nominalFF);
    //                                          ClosedLoopSlot.kSlot0);

    rollerMotorController.configure(
        rollerMotorConfig,
        com.revrobotics.ResetMode.kResetSafeParameters,
        com.revrobotics.PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(IntakeFeedIOInputs inOutData) {
    inOutData.connected = true;
    inOutData.velocity = (float) rollerEncoder.getVelocity();
    inOutData.motorTemperature = (float) rollerMotorController.getMotorTemperature();

    // Fault codes
    Faults armFaults = rollerMotorController.getFaults();
    inOutData.faultCan = armFaults.can;
    inOutData.faultTemperature = armFaults.temperature;
    inOutData.faultSensor = armFaults.sensor;
    inOutData.faultGateDriver = armFaults.gateDriver;
    inOutData.faultEscEeprom = armFaults.escEeprom;
    inOutData.faultFirmware = armFaults.firmware;

    // Outputs
    inOutData.busVoltage = (float) rollerMotorController.getBusVoltage();
    inOutData.outputDuty =
        (float) rollerMotorController.getAppliedOutput(); // -1 to 1 percent applied of bus voltage
    inOutData.outputCurrent = (float) rollerMotorController.getOutputCurrent();
    inOutData.outputVoltage = (float) rollerMotorController.getAppliedOutput() * 12.0f;
  }
  
  // @Override
  public void setVelocity(double rpm) {
   /* double adjustedRpm = rpm;
    if (rpm > kFeedMaxRpm) {
      adjustedRpm = kFeedMaxRpm;
    }
    else if (rpm < -kFeedMaxRpm) {
      adjustedRpm = -kFeedMaxRpm;
    }*/

    closedLoop.setSetpoint(rpm, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void setVoltage(double volts) {
    rollerMotorController.setVoltage(volts);
  }
}
