package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
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
    
    // Used to sycnronize control requests to the shooter motor paring
    private static final int kMotorPairControlUpdateTimeSyncHz = 50;

    private static final int kStatorCurrentLimit = 15;

    // TODO>>>..
    private static final double kMeasuredKv = 590.0;
    private static final double kS = 0.1435; // TODO tune Static mechanism friction

    private final TalonFX leaderTalon;
    private final TalonFXConfiguration leaderConfig;
    private final TalonFX followTalon;
    private final TalonFXConfiguration followConfig;

    private double currentFeedForward = 0.0;

    // Leader Control requests
    private DutyCycleOut openLoopRequest;
    private VelocityTorqueCurrentFOC velocityTorqueRequest;
    
    // Follower motor must always use the follow request otherwise hardware will break!!!
    private Follower followRequest;

  ShooterIOTalonFX(int leaderCanId, int followCanId) {
    
    // Setup common control request objects that will be reused for each loop iteration.

    openLoopRequest = new DutyCycleOut(0).withEnableFOC(true);

    // Needs motor config time request or something? TODO
    velocityTorqueRequest = new VelocityTorqueCurrentFOC(0.0)
      .withVelocity(0.0)
      .withSlot(0)
      .withUseTimesync(true)  // Use this with request updateFreq 0 hz
      .withUpdateFreqHz(0); // MotorConfig must have set freq rate to use this.

    // Use aligned follow request because the follower motor config is inverted already.
    // Even though motor config is already inverted, use the Opposed alignment config to
    // ensure that both motors spin towards the back of the robot. The overdrive gear will
    // increase rps and reverse direction so that the fuel is propelled out of the shooter.
    followRequest = new Follower(leaderCanId, MotorAlignmentValue.Opposed); 

    CurrentLimitsConfigs commonCurrentLimitsConfig = 
      new CurrentLimitsConfigs()
            .withStatorCurrentLimit(kStatorCurrentLimit)
            .withStatorCurrentLimitEnable(true);

    // Lead Motor (Robot left side, looking out to front of robot)
    // Motor spin is defined as looking at the face of the motor with the shaft pointing at observer
    // Due to the overdrive gears the shooter motor spin direction is intake, so the output gear will be
    // throw the fuel out.
    leaderTalon = new TalonFX(leaderCanId,TunerConstants.kCANBus);
    leaderConfig = new TalonFXConfiguration()
      .withCurrentLimits(commonCurrentLimitsConfig)
      .withMotorOutput(
        new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast)
                                 .withInverted(InvertedValue.CounterClockwise_Positive)
                                 .withControlTimesyncFreqHz(kMotorPairControlUpdateTimeSyncHz)
      );

    // Closed loop settings for velocity control
    leaderConfig.Slot0
        .withKS(0.1) // Add V output to overcome static friction
        .withKV(0.0) // Velocity target of 1 rps results in kV Volts output
        .withKP(0.0) // An error of 1 rps results in KP Volts output
        .withKI(0.0) // Avoid non-zero: No output for integrated error
        .withKD(0.0); // Output for error derivative

    // Apply leader config
    leaderTalon.getConfigurator().apply(leaderConfig);


    // Follow Motor (Robot right side, looking out to front of robot)
    followTalon = new TalonFX(followCanId,TunerConstants.kCANBus);
    followConfig = new TalonFXConfiguration()
      .withCurrentLimits(commonCurrentLimitsConfig)
      .withMotorOutput(
        new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast)
                                .withInverted(InvertedValue.Clockwise_Positive)
                                .withControlTimesyncFreqHz(kMotorPairControlUpdateTimeSyncHz)
      );
    followTalon.getConfigurator().apply(followConfig); // Apply follow config

  }

  // Update shooter motor inputs
  @Override
  public void updateInputs(ShooterIOInputs inOutData)
  {
    inOutData.connected = leaderTalon.isAlive() && followTalon.isAlive();
    inOutData.motorVelocity = leaderTalon.getVelocity().getValueAsDouble() * 60.0f; // CTR provides RPS convert to RPM
    inOutData.flywheelVelocity = inOutData.motorVelocity * Flywheel.kDriveToOutputGearRatio;
    inOutData.leaderMotorTemperature = (float) leaderTalon.getDeviceTemp().getValueAsDouble();
    inOutData.followMotorTemperature = (float) followTalon.getDeviceTemp().getValueAsDouble();
    inOutData.maxFlywheelSpeedHit = inOutData.flywheelVelocity > Flywheel.kMaxShooterRPM;
    
    // Fault codes
    inOutData.faultSupplyUnderVoltage = leaderTalon.getFault_Undervoltage().getValue() || followTalon.getFault_Undervoltage().getValue();
    inOutData.faultBridgeBrownout = leaderTalon.getFault_BridgeBrownout().getValue() || followTalon.getFault_BridgeBrownout().getValue();
    inOutData.faultTemperature = leaderTalon.getFault_DeviceTemp().getValue() || followTalon.getFault_DeviceTemp().getValue();
    inOutData.faultControllerTemperature = leaderTalon.getFault_ProcTemp().getValue() || followTalon.getFault_ProcTemp().getValue();
    inOutData.faultHardwareLeaderMotor = leaderTalon.getFault_Hardware().getValue();
    inOutData.faultHardwareFollowMotor = followTalon.getFault_Hardware().getValue();
    inOutData.faultStatorCurrentLimitLeaderMotor = followTalon.getFault_StatorCurrLimit().getValue();
    inOutData.faultStatorCurrentLimitFollowMotor = followTalon.getFault_StatorCurrLimit().getValue();

    // Outputs
    inOutData.busVoltage = (float) leaderTalon.getSupplyCurrent().getValueAsDouble();
    inOutData.outputDuty = (float) leaderTalon.getDutyCycle().getValueAsDouble(); // -1 to 1 percent applied of bus voltage
    inOutData.outputCurrent = (float) leaderTalon.getStatorCurrent().getValueAsDouble();
    inOutData.outputVoltage = (float) leaderTalon.getMotorVoltage().getValueAsDouble();

    inOutData.feedForward = currentFeedForward;
  }

  // Set shooter target velocity in RPM, using closed loop with feedforward.
  @Override
  public void setVelocity(double rpm) {

    // Adjust for output overdrive gearing. The request is relative to the desired flywheel output rpm.
    // This will decrease the requested motor RPM so that the output flywheel is at the requested rpm.
    double adjustedRpm = Flywheel.kOutputToDriveGearRatio * rpm;
    /*
    Raw feed forward style
    final double flywheelFeedForward = 1.0 / kMeasuredKv; // Measured kV 590 of flywheel
    final double scaledFeedForward = flywheelFeedForward * rpm + kS;
    currentFeedForward = scaledFeedForward;
    closedLoop.setSetpoint(rpm, ControlType.kVelocity,ClosedLoopSlot.kSlot0, currentFeedForward);*/
    
    
    velocityTorqueRequest.Velocity = adjustedRpm / 60.0; // CTR uses revolutions per second

    // Control requests must be sent in pairs to control the leader and follower together.
    leaderTalon.setControl(velocityTorqueRequest);
    followTalon.setControl(followRequest);
    
  }

  @Override
  public void setVoltage(double volts) {
    double duty = volts / Constants.kNominalVoltage;
    openLoopRequest.Output = duty;
    
    // Control requests must be sent in pairs to control the leader and follower together.
    leaderTalon.setControl(openLoopRequest);
    followTalon.setControl(followRequest);
  }

}