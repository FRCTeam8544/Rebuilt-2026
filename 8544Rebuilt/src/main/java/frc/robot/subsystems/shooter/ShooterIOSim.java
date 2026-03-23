package frc.robot.subsystems.shooter;

import frc.robot.Constants;
import frc.robot.subsystems.shooter.Shooter.Flywheel;

/**
 * Simulated shooter IO. Models flywheel velocity as proportional to applied voltage — no physics
 * engine needed, just enough fidelity for overspeed protection and launch speed scaling.
 */
public class ShooterIOSim implements ShooterIO {

  private double appliedVoltageVolts = 0.0;

  @Override
  public void updateInputs(ShooterIOInputs inOutData) {
    inOutData.connected = true;

    // Proportional motor velocity: fraction of free speed based on applied voltage
    double fractionOfMax = appliedVoltageVolts / Constants.kNominalVoltage;
    inOutData.motorVelocity = Constants.KrakenX60.freeSpeedRPM * fractionOfMax;
    inOutData.flywheelVelocity = inOutData.motorVelocity * Flywheel.kDriveToOutputGearRatio;

    inOutData.busVoltage = (float) Constants.kNominalVoltage;
    inOutData.outputVoltage = (float) appliedVoltageVolts;
    inOutData.outputDuty = (float) fractionOfMax;
    inOutData.outputCurrent = 0.0f;
  }

  @Override
  public void setVelocity(double rpm) {
    // Convert motor RPM setpoint to equivalent voltage
    double equivalentVoltage = (rpm / Constants.KrakenX60.freeSpeedRPM) * Constants.kNominalVoltage;
    appliedVoltageVolts = equivalentVoltage;
  }

  @Override
  public void setVoltage(double volts) {
    appliedVoltageVolts = volts;
  }
}
