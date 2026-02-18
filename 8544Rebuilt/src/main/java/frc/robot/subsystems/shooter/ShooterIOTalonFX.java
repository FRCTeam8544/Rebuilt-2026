package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;

// Talon FX paired with Kraken X60 motors.

public class ShooterIOTalonFX implements ShooterIO {

    
    private static final int stallLimit = 60;

    private static final double kMeasuredKv = 590.0;
    private static final double kS = 0.1435; // Static mechanism friction

    private double currentFeedForward = 0.0;

    private final TalonFX leaderTalon;
    private final TalonFX followTalon;

  ShooterIOTalonFX(int leaderCanId, int followCanId) {
    leadTalon = new TalonFX(leaderCanId);
    followTalk = new TalonFX(followCanId);
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
    leaderMotorController.setVoltage(volts);
  }

}