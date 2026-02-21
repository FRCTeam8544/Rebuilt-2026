package frc.robot.subsystems.shooter;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.Faults;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkClosedLoopController;

import frc.robot.Constants.NeoVortex;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.Shooter.FeedWheel;
import pabeles.concurrency.IntObjectConsumer;

public class FeedIOFlex implements FeedIO {
  
    private static final int stallLimit = 40;

    private static final double kMeasuredKv = 540.0;  // Was 540
    private static final double kS = 0.95; // Static mechanism friction

    private double currentFeedForward = 0.0;

    private final SparkFlex motorController;    
    private final RelativeEncoder motorEncoder;
    private final SparkClosedLoopController closedLoop;
    private final SparkFlexConfig motorConfig;

  public FeedIOFlex(int canId) {
    motorController = new SparkFlex(canId, MotorType.kBrushless);
    motorEncoder = motorController.getEncoder();
    closedLoop = motorController.getClosedLoopController();
    
    motorConfig = new SparkFlexConfig();
    motorConfig.idleMode(IdleMode.kBrake);
    motorConfig.smartCurrentLimit(stallLimit);
    motorConfig.voltageCompensation(12);
    motorConfig.softLimit.forwardSoftLimitEnabled(false);
    motorConfig.softLimit.reverseSoftLimitEnabled(false);
    motorConfig.encoder.positionConversionFactor(1/20.0); // 20 to 1 gearbox
    motorConfig.encoder.velocityConversionFactor(1/20.0); // 20 to 1 gearbox

    motorConfig.closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // Velocity control
          .p(0.00002, ClosedLoopSlot.kSlot0)
          .i(0.00000, ClosedLoopSlot.kSlot0)
          .d(0.00000, ClosedLoopSlot.kSlot0);
    motorConfig.closedLoop.feedForward.kV(NeoVortex.nominalFF,
                                          ClosedLoopSlot.kSlot0);
                                          
    motorController.configure(motorConfig, 
                              com.revrobotics.ResetMode.kResetSafeParameters,
                              com.revrobotics.PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(FeedIOInputs inOutData)
  {
    inOutData.connected = true;
    inOutData.motorVelocity = motorEncoder.getVelocity();
    inOutData.wheelVelocity = inOutData.motorVelocity * FeedWheel.kDriveToOutputGearRatio;
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
    inOutData.outputVoltage = (float) motorController.getAppliedOutput() * 12.0f;

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
    inOutData.voltageSetPoint(volts);
    inOutData.velocitySetPoint(0);
    motorController.setVoltage(volts);
  }

}