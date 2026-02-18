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

import frc.robot.Constants;
import frc.robot.generated.TunerConstants;

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
    private VelocityTorqueCurrentFOC velocityTorqueRequest = new VelocityTorqueCurrentFOC(0.0);
    
    // Follower motor must always use the follow request otherwise hardware will break!!!
    private Follower followRequest = new Follower(leaderCanId, MotorAlignmentValue.Opposed);
    
  ShooterIOTalonFX(int leaderCanId, int followCanId) {
    
    // Common configuration
    this.leaderCanId = leaderCanId;
    this.followCanId = followCanId;
    
    // Needs motor config time request or something?
   // velocityTorqueRequest.UseTimesync = true;
   // velocityTorqueRequest.UpdateFreqHz = 0;

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
  }

}