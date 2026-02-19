package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase.Faults;

import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.shooter.Shooter.Flywheel;

// Talon FX paired with Kraken X60 motors.

public class ShooterIOTalonFX implements ShooterIO {
    
    
    private static final int kCurrentLimit = 80;
    private static final double kMeasuredKv = 590.0;
    private static final double kS = 0.1435; // Static mechanism friction

    private final int leaderCanId;
    private final TalonFX leaderTalon;
    private final TalonFXConfiguration leaderConfig;
    private final int followCanId;
    private final TalonFX followTalon;
    private final TalonFXConfiguration followConfig;

    private double currentFeedForward = 0.0;

    // Leader Control requests
    private DutyCycleOut openLoopRequest = new DutyCycleOut(0);
    private VelocityTorqueCurrentFOC velocityTorqueRequest;
    
    // Follower motor must always use the follow request otherwise hardware will break!!!
    private Follower followRequest;

  ShooterIOTalonFX(int leaderCanId, int followCanId) {
    
    // Common configuration
    this.leaderCanId = leaderCanId;
    this.followCanId = followCanId;
    
    // Needs motor config time request or something?
    velocityTorqueRequest = new VelocityTorqueCurrentFOC(0.0);
   // velocityTorqueRequest.UseTimesync = true;
   // velocityTorqueRequest.UpdateFreqHz = 0;

    // Use aligned follow request because the follower motor config is inverted already.
    // Positive voltage for either motor will result synchronized movement.
    followRequest = new Follower(leaderCanId, MotorAlignmentValue.Opposed); 

    CurrentLimitsConfigs commonCurrentLimitsConfig = 
      new CurrentLimitsConfigs()
            .withStatorCurrentLimit(kCurrentLimit)
            .withStatorCurrentLimitEnable(true);

    // Lead Motor (Robot left side, looking out to front of robot)
    // Motor spin is defined as looking at the face of the motor with the shaft pointing at observer
    // Due to the overdrive gears the shooter motor spin direction is intake, so the output gear will be
    // throw the fuel out.
    leaderTalon = new TalonFX(leaderCanId,TunerConstants.kCANBus);
    leaderConfig = new TalonFXConfiguration()
      .withCurrentLimits(commonCurrentLimitsConfig)
      .withMotorOutput(
        new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast).
                                 withInverted(InvertedValue.CounterClockwise_Positive)
      );
    leaderTalon.getConfigurator().apply(leaderConfig); // Apply leader config


    // Follow Motor (Robot right side, looking out to front of robot)
    followTalon = new TalonFX(followCanId,TunerConstants.kCANBus);
    followConfig = new TalonFXConfiguration()
      .withCurrentLimits(commonCurrentLimitsConfig)
      .withMotorOutput(
        new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast)
                                .withInverted(InvertedValue.Clockwise_Positive)
      );
    followTalon.getConfigurator().apply(followConfig); // Apply follow config

  }

  
  @Override
  public void updateInputs(ShooterIOInputs inOutData)
  {
    inOutData.connected = leaderTalon.isAlive() && followTalon.isAlive();
    inOutData.motorVelocity = (float) leaderTalon.getVelocity().getValueAsDouble();
    inOutData.flywheelVelocity = inOutData.motorVelocity * (float) Flywheel.kDriveToOutputGearRatio;
    inOutData.leaderMotorTemperature = (float) leaderTalon.getDeviceTemp().getValueAsDouble();
    inOutData.followMotorTemperature = (float) followTalon.getDeviceTemp().getValueAsDouble();

    // Fault codes
   /* Faults leaderFaults = leaderMotorController.getFaults();
    Faults followFaults = leaderMotorController.getFaults();
    inOutData.faultCan = leaderFaults.can || followFaults.can;
    inOutData.faultTemperature = leaderFaults.temperature || followFaults.temperature;
    inOutData.faultSensor = leaderFaults.sensor || followFaults.sensor;
    inOutData.faultGateDriver = leaderFaults.gateDriver || followFaults.gateDriver;
    inOutData.faultEscEeprom = leaderFaults.escEeprom || followFaults.escEeprom;
    inOutData.faultFirmware = leaderFaults.firmware || followFaults.firmware;
*/
    // Outputs
    inOutData.busVoltage = (float) leaderTalon.getSupplyCurrent().getValueAsDouble();
    inOutData.outputDuty = (float) leaderTalon.getDutyCycle().getValueAsDouble(); // -1 to 1 percent applied of bus voltage
    inOutData.outputCurrent = (float) leaderTalon.getStatorCurrent().getValueAsDouble();
    inOutData.outputVoltage = (float) leaderTalon.getMotorVoltage().getValueAsDouble();

    inOutData.feedForward = currentFeedForward;
  }

  @Override
  public void setVelocity(double rpm) {

    // Adjust for output overdrive gearing. The request is relative to the desired flywheel output rpm.
    // This will decrease the requested motor RPM so that the output flywheel is at the requested rpm.
 //   double adjustedRpm = kOutputToDriveGearRatio * rpm;
    /*
    final double flywheelFeedForward = 1.0 / kMeasuredKv; // Measured kV 590 of flywheel
    final double scaledFeedForward = flywheelFeedForward * rpm + kS;
    currentFeedForward = scaledFeedForward;
    closedLoop.setSetpoint(rpm, ControlType.kVelocity,ClosedLoopSlot.kSlot0, currentFeedForward);*/
    rpm = 0.0; // TODO
    
    velocityTorqueRequest.Velocity = rpm;
    leaderTalon.setControl(velocityTorqueRequest);
    followTalon.setControl(followRequest);
    
  }

  @Override
  public void setVoltage(double volts) {
    double duty = volts / Constants.kNominalVoltage;
    openLoopRequest.Output = duty;
    leaderTalon.setControl(openLoopRequest);
    followTalon.setControl(followRequest);
  }

}