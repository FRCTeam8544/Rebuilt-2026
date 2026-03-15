package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {


  public static final int intakeCanId = 28;
  public final IntakeIO intakeIO;
 
  public final IntakeIOInputsAutoLogged intakeInputs = new IntakeIOInputsAutoLogged();
  

  public BooleanSupplier isIntaking =
    () -> {
      return intakeInputs.velocity > 1.0; // RPM
    };

  public DoubleSupplier intakeRpmSupplier =
    () -> {
      return intakeInputs.velocity;
    };

  public DoubleSupplier intakeRpmSetPoint = 
    () -> {
      return intakeInputs.velocitySetPoint;
    };

  public Intake(IntakeIO intakeIO) {
    this.intakeIO = intakeIO;
  }

  // Open loop control

  public void runOpenLoop(double duty) {
    double adjustedDuty = duty;
    if (duty > 1.0) {
      adjustedDuty = 1.0;
    }
    else if (duty < -1.0) {
      adjustedDuty = -1.0;
    }

    intakeInputs.voltageSetPoint = adjustedDuty * Constants.kNominalVoltage;
    intakeInputs.velocitySetPoint = 0.0;

    intakeIO.setVoltage(intakeInputs.voltageSetPoint);
  }

  public void stopOpenLoop() {
    runOpenLoop(0);
  }

  // Closed loop control 

  public void runAtRpm(double rpm) {
    intakeInputs.velocitySetPoint = rpm;
    intakeInputs.voltageSetPoint = 0.0;
    intakeIO.setVelocity(intakeInputs.velocitySetPoint);
  }
  
  public void stopMotors() {
    intakeInputs.voltageSetPoint = 0;
    intakeInputs.velocitySetPoint = 0;
    intakeIO.setVoltage(0);
  }

  @Override
  public void periodic() {
    intakeIO.updateInputs(intakeInputs);
    Logger.processInputs("Intake", intakeInputs);
  }
}
