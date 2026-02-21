package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake.*;


import static edu.wpi.first.units.Units.Rotations;

import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {


  public static final double kMaxIntakeRPM = 2000; //100:1 gearbox


  public static final int armCanId = 27;
  public static final int feedCanId = 28;

  public final IntakeIO intakeArmIO;
  public final IntakeIOInputsAutoLogged intakeArmInputs = new IntakeIOInputsAutoLogged();

  private final IntakeFeedIO intakeFeedIO;
  private final IntakeFeedIOInputsAutoLogged intakeFeedInputs = new IntakeFeedIOInputsAutoLogged();

  public Intake() {
    this.intakeArmIO = new IntakeIOMax(armCanId);
    this.intakeFeedIO = new IntakeFeedIOMax(feedCanId);
  }

  public void runArmOpenLoop(double duty) {
    
    final double forwardLimit = 0.86; // Zero is "straight up" 870
    final double backwardLimit = 0.07; // Stow position

    double adjustedDuty = duty;
    adjustedDuty = Math.min(adjustedDuty, 1.0);
    adjustedDuty = Math.max(adjustedDuty, -1.0);

    intakeArmInputs.voltageSetPoint = adjustedDuty * Constants.Neo.nominalVoltage;
    intakeArmInputs.positionSetPoint = 0.0;

    if ((intakeArmInputs.position < forwardLimit) && 
        (intakeArmInputs.position > backwardLimit) ) {
      intakeArmIO.setVoltage(intakeArmInputs.voltageSetPoint);
    }
  }

  public void runFeedOpenLoop(double duty) {
    double adjustedDuty = duty;
  //  adjustedDuty = Math.min(adjustedDuty, 1.0);
    //adjustedDuty = Math.max(adjustedDuty, -1.0);
    if (duty > 1.0) {
      adjustedDuty = 1.0;
    }
    else if (duty < -1.0) {
      adjustedDuty = -1.0;
    }

    intakeFeedInputs.voltageSetPoint = adjustedDuty * Constants.Neo.nominalVoltage;
    intakeFeedInputs.velocitySetPoint = 0.0;

    intakeFeedIO.setVoltage(intakeFeedInputs.voltageSetPoint);
  }

  public void runIntakeArm(double rotations) {
    intakeArmInputs.positionSetPoint = rotations;
    intakeArmInputs.voltageSetPoint = 0.0;
    intakeArmIO.setPosition(intakeArmInputs.positionSetPoint);
  } 

  public void runIntakeFeed(double rpm) {
    intakeFeedInputs.velocitySetPoint = rpm;
    intakeFeedIO.setVelocity(intakeFeedInputs.velocitySetPoint);
  }
  
  public void stopFeed() {
    intakeFeedInputs.voltageSetPoint = 0;
    intakeFeedInputs.velocitySetPoint = 0;
    intakeFeedIO.setVoltage(0);
    //intakeFeedIO.setVelocity(intakeFeedInputs.velocitySetPoint);
  }

  public void holdArmPosition() {
    intakeArmInputs.positionSetPoint = intakeArmInputs.position;
    intakeArmIO.setPosition(intakeArmInputs.position);
  }

  public void stopOpenLoop() {
    intakeArmIO.setVoltage(0);
    intakeFeedIO.setVoltage(0);
  }

  @Override
  public void periodic() {
    intakeArmIO.updateInputs(intakeArmInputs);
    intakeFeedIO.updateInputs(intakeFeedInputs);
    Logger.processInputs("Intake/Arm", intakeArmInputs);
    Logger.processInputs("Intake/Feed", intakeFeedInputs);
    
  }
}
