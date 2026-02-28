package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake.*;


import static edu.wpi.first.units.Units.Rotations;

import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {


  public static final double kMaxIntakeRPM = 2000; //100:1 gearbox


  public static final int intakeCanId = 28;
  public final IntakeIO intakeIO;
 
  public final IntakeIOInputsAutoLogged intakeInputs = new IntakeIOInputsAutoLogged();


  public Intake() {
   
    this.intakeIO = new IntakeIOMax(intakeCanId);
  }

  public void runIntakeOpenLoop(double duty) {
    double adjustedDuty = duty;
  //  adjustedDuty = Math.min(adjustedDuty, 1.0);
    //adjustedDuty = Math.max(adjustedDuty, -1.0);
    if (duty > 1.0) {
      adjustedDuty = 1.0;
    }
    else if (duty < -1.0) {
      adjustedDuty = -1.0;
    }

    intakeInputs.voltageSetPoint = adjustedDuty * Constants.Neo.nominalVoltage;
    intakeInputs.velocitySetPoint = 0.0;

    intakeIO.setVoltage(intakeInputs.voltageSetPoint);
  }


  public void runIntakeFeed(double rpm) {
    intakeInputs.velocitySetPoint = rpm;
    intakeIO.setVelocity(intakeInputs.velocitySetPoint);
  }
  
  public void stopIntake() {
    intakeInputs.voltageSetPoint = 0;
    intakeInputs.velocitySetPoint = 0;
    intakeIO.setVoltage(0);
    //intakeFeedIO.setVelocity(intakeFeedInputs.velocitySetPoint);
  }


  public void stopOpenLoop() {
    intakeIO.setVoltage(0);
  }

  @Override
  public void periodic() {
    intakeIO.updateInputs(intakeInputs);
    Logger.processInputs("Intake/Feed", intakeInputs);
    
  }
}
