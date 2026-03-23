package frc.robot.subsystems.Intake;

import frc.robot.Constants;
import org.ironmaple.simulation.IntakeSimulation;

/**
 * Maple-sim intake IO implementation. Uses IntakeSimulation for game piece detection and simulates
 * roller velocity proportional to applied voltage.
 */
public class IntakeIOSim implements IntakeIO {

  private static final double VOLTAGE_DEADBAND = 0.5;

  private final IntakeSimulation intakeSimulation;

  private double appliedVoltageVolts = 0.0;
  private boolean intakeRunning = false;

  /**
   * Creates a new IntakeIOSim.
   *
   * @param intakeSimulation the maple-sim IntakeSimulation instance
   */
  public IntakeIOSim(IntakeSimulation intakeSimulation) {
    this.intakeSimulation = intakeSimulation;
  }

  @Override
  public void updateInputs(IntakeIOInputs inOutData) {
    inOutData.connected = true;

    // Simulate roller velocity proportional to voltage
    double fractionOfMax = appliedVoltageVolts / Constants.kNominalVoltage;
    inOutData.velocity = (float) (kMaxIntakeRPM * fractionOfMax);

    // Report simulated outputs
    inOutData.busVoltage = (float) Constants.kNominalVoltage;
    inOutData.outputVoltage = (float) appliedVoltageVolts;
    inOutData.outputDuty = (float) (appliedVoltageVolts / Constants.kNominalVoltage);
    inOutData.outputCurrent = 0.0f;

    // Game piece state from maple-sim
    inOutData.fuelCount = intakeSimulation.getGamePiecesAmount();
    inOutData.hasFuel = inOutData.fuelCount > 0;
  }

  @Override
  public void setVoltage(double volts) {
    appliedVoltageVolts = volts;

    if (Math.abs(volts) > VOLTAGE_DEADBAND && !intakeRunning) {
      intakeSimulation.startIntake();
      intakeRunning = true;
    } else if (Math.abs(volts) <= VOLTAGE_DEADBAND && intakeRunning) {
      intakeSimulation.stopIntake();
      intakeRunning = false;
    }
  }

  @Override
  public void setVelocity(double rpm) {
    double equivalentVoltage = (rpm / kMaxIntakeRPM) * Constants.kNominalVoltage;
    setVoltage(equivalentVoltage);
  }
}
