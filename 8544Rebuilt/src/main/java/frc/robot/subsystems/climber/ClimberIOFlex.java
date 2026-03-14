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
import frc.robot.subsystems.climber.*;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.units.measure.Angle;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import frc.robot.Constants;
import frc.robot.generated.TunerConstants;

public class ClimberIOFlex implements ClimberIO {

    private static final double nominalVoltage = Constants.NeoVortex.nominalVoltage;
    private static final double nominalFF = Constants.NeoVortex.nominalFF;
    private static final double Ks = 0.120;
    private static final int stallLimit = 40;
    
    private final SparkFlex motorController;
    private final SparkFlexConfig motorConfig;

    private final CANcoder cancoder;
    private final CANcoderConfiguration cancoderConfig;

    // Converts from Cancoder position 0 to N rotations to the hook rotations
    public final double hookPositionToEncoderPositionRatio = 2.0; // TODO tune!!
    public final double encoderToHookPositionRatio = 1.0 / hookPositionToEncoderPositionRatio;

  public ClimberIOFlex(int canId, int encoderCanId) {
    motorController = new SparkFlex(canId, MotorType.kBrushless);

    cancoder = new CANcoder(encoderCanId, TunerConstants.kCANBus);
    cancoderConfig = new CANcoderConfiguration()
        .withMagnetSensor(
            new MagnetSensorConfigs()
              .withSensorDirection(SensorDirectionValue.Clockwise_Positive) // TODO?
        );
   // cancoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinusHalf;
    cancoder.getConfigurator().apply(cancoderConfig);

    // Spark flex controller defaults to counter clockwise positive rotation of the shaft.
    // Counter clockwise motion of the motor will cause the hooks, due to pulley, to bring the
    // hooks in towards the front of the robot. Since zero will be over the shooter and
    // the encoder is set to positive clockwise spin, the spark flex will need "inverted" voltage.
    // The motor will spin Clockwise with positive voltage that will be extend the hooks out from the
    // robot.
    motorConfig = new SparkFlexConfig();
    motorConfig.idleMode(IdleMode.kBrake);
    motorConfig.smartCurrentLimit(stallLimit);
    motorConfig.voltageCompensation(nominalVoltage);
    motorConfig.inverted(true); // Clockwise positive voltage rotation
    motorConfig.softLimit.forwardSoftLimitEnabled(false);
    motorConfig.softLimit.reverseSoftLimitEnabled(false);

    // REV encoder will no longer be used.
    /*motorConfig.closedLoop
          .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
          // Position Control
          .p(150.00, ClosedLoopSlot.kSlot0)
          .i(0.00000, ClosedLoopSlot.kSlot0)
          .outputRange(-1, 1, ClosedLoopSlot.kSlot0)
          .d(0.00000, ClosedLoopSlot.kSlot0);
          
    motorConfig.closedLoop.feedForward.kS(Ks, ClosedLoopSlot.kSlot0);
          
    motorConfig.closedLoop.feedForward.kV(12 * nominalFF,
                                          ClosedLoopSlot.kSlot0);*/
   
                                          
    motorController.configure(motorConfig, 
                              com.revrobotics.ResetMode.kResetSafeParameters,
                              com.revrobotics.PersistMode.kPersistParameters);
  }

  public void updateInputs(ClimberIOInputs inOutData) {
    inOutData.connected = true;
    inOutData.motorTemperature = (float) motorController.getMotorTemperature();
    inOutData.velocity = cancoder.getVelocity().getValueAsDouble() / 60.0; // rps -> rpm
    inOutData.encoderPosition = cancoder.getPosition().getValueAsDouble();
    inOutData.position = (float) inOutData.encoderPosition / encoderToHookPositionRatio;

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

  public void setBrakeMode(boolean enableMotorBrake) {

    if (enableMotorBrake) {
      motorConfig.idleMode(IdleMode.kBrake);
    }
    else {
      motorConfig.idleMode(IdleMode.kCoast);
    }

    // Do not persist the configuration. It is too slow to do during operation
    motorController.configure(motorConfig, 
                              com.revrobotics.ResetMode.kNoResetSafeParameters,
                              com.revrobotics.PersistMode.kNoPersistParameters);
  }

  public void setPosition(double rotations) {

    // TODO USE WPILib PID loop implementation to control the climber
    // USE the "hook" position from inOutData as that is a synthetic absolute encoder with range 0 to 1
    // That will not wrap.
   // closedLoop.setSetpoint(rotations, ControlType.kPosition,ClosedLoopSlot.kSlot0);
  }

  public void setVoltage(double volts) {
    motorController.setVoltage(volts);
  }
}
