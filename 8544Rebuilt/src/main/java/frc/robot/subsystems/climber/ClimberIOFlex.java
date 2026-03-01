package frc.robot.subsystems.climber;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.Faults;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.units.measure.Angle;

import com.ctre.phoenix6.hardware.CANcoder;

import frc.robot.Constants;
import frc.robot.generated.TunerConstants;

public class ClimberIOFlex implements ClimberIO {

    private static final double nominalVoltage = Constants.NeoVortex.nominalVoltage;
    private static final double nominalFF = Constants.NeoVortex.nominalFF;
    private static final double Ks = 0.120;
    private static final int stallLimit = 40;
    private static double climberPosition = 0;
    //private final double cancoderPosition=0;
    
    private final SparkFlex motorController;    
    private final AbsoluteEncoder motorEncoder;
    private final SparkClosedLoopController closedLoop;
    private final SparkFlexConfig motorConfig;
    private final CANcoder cancoder;



  public ClimberIOFlex(int canId) {
    motorController = new SparkFlex(canId, MotorType.kBrushless);
    motorEncoder = motorController.getAbsoluteEncoder();// TODO should be external encoder??
    closedLoop = motorController.getClosedLoopController();

    cancoder = new CANcoder(31, TunerConstants.kCANBus);
    climberPosition = cancoder.getPosition().getValueAsDouble();

    motorConfig = new SparkFlexConfig();
    motorConfig.idleMode(IdleMode.kBrake);
    motorConfig.smartCurrentLimit(stallLimit);
    motorConfig.voltageCompensation(nominalVoltage);
    motorConfig.softLimit.forwardSoftLimitEnabled(false); //was true
    motorConfig.softLimit.reverseSoftLimitEnabled(false);
    motorConfig.closedLoop.positionWrappingEnabled(true);
 //   motorConfig.softLimit.forwardSoftLimit(0.8);
  //  motorConfig.softLimit.reverseSoftLimit(0.2);
  //  motorConfig.encoder.positionConversionFactor(1.0/100.0);
  //  motorConfig.encoder.velocityConversionFactor(1.0/100.0);

    motorConfig.closedLoop
          .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
          // Position Control
          .p(150.00, ClosedLoopSlot.kSlot0)
          .i(0.00000, ClosedLoopSlot.kSlot0)
          .outputRange(-1, 1, ClosedLoopSlot.kSlot0)
          .d(0.00000, ClosedLoopSlot.kSlot0);
          
       //   motorConfig.closedLoop.feedForward.kS(Ks, ClosedLoopSlot.kSlot0);
          
 //   motorConfig.closedLoop.feedForward.kV(12 * nominalFF,
  //                                        ClosedLoopSlot.kSlot0);
   
                                          
    motorController.configure(motorConfig, 
                              com.revrobotics.ResetMode.kResetSafeParameters,
                              com.revrobotics.PersistMode.kPersistParameters);
  }

  public void updateInputs(ClimberIOInputs inOutData) {
    inOutData.connected = true;
    inOutData.velocity = (float) motorEncoder.getVelocity();
    inOutData.position = (float) motorEncoder.getPosition();
    inOutData.motorTemperature = (float) motorController.getMotorTemperature();
    //cancoder
    inOutData.climberPosition = (float) climberPosition;

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
    inOutData.outputVoltage = (float) (motorController.getAppliedOutput() * nominalVoltage);

  }

  public void setPosition(double rotations) {

    closedLoop.setSetpoint(rotations, ControlType.kPosition,ClosedLoopSlot.kSlot0);
  }

  public void setVoltage(double volts) {
    motorController.setVoltage(volts);
  }
}
