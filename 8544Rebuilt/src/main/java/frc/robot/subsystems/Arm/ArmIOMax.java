package frc.robot.subsystems.Arm;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.Faults;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants;
import frc.robot.util.SparkUtil;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.FeedbackSensor;

public class ArmIOMax implements ArmIO {

  private static final int stallLimit = 10;
  private static final int configAttempts = 5;

  // Get the zero offset value from rev client TODO
  // The zero offset is added to the hardware encoder value that is returned by getPosition()
  // positionCorrectionFactor and velocityCorrectionFactor are ignored for this.
  private static final double absEncoderZeroOffset = 0.0; 

  private final SparkMax armMotorController;
  private final SparkClosedLoopController closedLoopController;

  private final AbsoluteEncoder armEncoder;
  private final SparkMaxConfig armMotorConfig;


  public ArmIOMax(int armCanId) {
    armMotorController = new SparkMax(armCanId, MotorType.kBrushless);
    closedLoopController = armMotorController.getClosedLoopController();
    armEncoder = armMotorController.getAbsoluteEncoder();

    // Spark max defaults to clockwise rotation from viewing the motor shaft with positive voltage
    // Due to the pully assembly from the motor to the arm, clockwise motor rotation will cause
    // the retract the intake ARM towards the shooter. counterclockwise will push the arm out.
    armMotorConfig = new SparkMaxConfig();
    armMotorConfig.inverted(false);
    armMotorConfig.idleMode(IdleMode.kBrake);
    armMotorConfig.smartCurrentLimit(stallLimit);
    armMotorConfig.voltageCompensation(12);

    // Encoder
    armMotorConfig.absoluteEncoder.inverted(false);
   // armMotorConfig.absoluteEncoder.zeroOffset(absEncoderZeroOffset)
   // armMotorConfig.encoder.positionConversionFactor(3); // Pullys are 1 motor to 3 arm rotations
  //  armMotorConfig.encoder.velocityConversionFactor(3);

    // Limits
    armMotorConfig.softLimit.forwardSoftLimitEnabled(true);
    armMotorConfig.softLimit.forwardSoftLimit(kArmForwardLimit);
    armMotorConfig.softLimit.reverseSoftLimitEnabled(true);
    armMotorConfig.softLimit.reverseSoftLimit(kArmReverseLimit);

    // Signals
    armMotorConfig.signals.absoluteEncoderPositionAlwaysOn(true);
    armMotorConfig.signals.absoluteEncoderVelocityAlwaysOn(true);

    // Closed loop settings
    armMotorConfig
        .closedLoop
        .positionWrappingEnabled(false)
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        // Position control
        .p(4, ClosedLoopSlot.kSlot0) //was 5
        .i(0.00000, ClosedLoopSlot.kSlot0)
        .d(0.3, ClosedLoopSlot.kSlot0); //0.00001

    
    // armMotorConfig.closedLoop.feedForward.kS(kS);
    //  armMotorConfig.closedLoop.feedForward.kV(Constants.Neo.nominalFF);
    //                                          ClosedLoopSlot.kSlot0);

    SparkUtil.tryUntilOk(
        armMotorController, 
        configAttempts,
        () -> { return armMotorController.configure(
                    armMotorConfig,
                    com.revrobotics.ResetMode.kResetSafeParameters,
                    com.revrobotics.PersistMode.kPersistParameters); } );
    
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
    inOutData.outputVoltage = (float) (armMotorController.getAppliedOutput() * Constants.kNominalVoltage);
    inOutData.outputPower = inOutData.outputCurrent * inOutData.outputVoltage;
    inOutData.outputTorque = inOutData.outputCurrent * (float) Constants.Neo.kTorque_mN_per_Amp;
  }
  
  // @Override
  public void setPosition(double rotations) {
    closedLoopController.setSetpoint(rotations, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void setVoltage(double volts) {
    armMotorController.setVoltage(volts);
  }

}
