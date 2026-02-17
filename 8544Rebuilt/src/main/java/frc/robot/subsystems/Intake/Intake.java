package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake.*;


import static edu.wpi.first.units.Units.Rotations;

import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {


  public static final double kMaxIntakeRPM = 2000; //100:1 gearbox


  public static final int armCanId = 27;

  public final IntakeIO IntakeIO;
  public final IntakeIOInputsAutoLogged IntakeInputs = new IntakeIOInputsAutoLogged();

  // private final IntakeFeedIO IntakeFeedIO;

  private double tuneShootVoltage = 0.0;
  private final double tuneShootVoltStep = 0.25 / 50; // 1/4 volt per second

  public Intake() {
    this.IntakeIO = new IntakeIOMax(armCanId);
  }

  public void tuneIncreaseVoltage() {
    tuneShootVoltage += tuneShootVoltStep;
    if (tuneShootVoltage > 12.0) {
      tuneShootVoltage = 12.0;
    }
  }

  public void tuneDecreaseVoltage() {
    tuneShootVoltage -= tuneShootVoltStep;
    if (tuneShootVoltage < 0.0) {
      tuneShootVoltage = 0.0;
    }
  }


  public void runIntakeOpenLoop(boolean extend) {
 double sign = 1.0;
 if ( extend) {sign = 1.0;}
  else{ sign = -1.0;}

    IntakeInputs.voltageSetPoint = tuneShootVoltage * sign;
    IntakeInputs.positionSetPoint = 0.0;
    
    IntakeIO.setVoltage(IntakeInputs.voltageSetPoint);
  }




  public void runIntake(double rotations) {


    IntakeInputs.positionSetPoint = rotations;
    IntakeIO.setPosition(IntakeInputs.positionSetPoint);
  } 
  
  
  public void holdPosition() {


    IntakeInputs.positionSetPoint = IntakeInputs.position;
    IntakeIO.setPosition(IntakeInputs.position);
  }

  public void stopOpenLoop() {
    // runFeedOpenLoop(0.0);
   IntakeInputs.voltageSetPoint = 0;
   IntakeIO.setVoltage(0);
  }

  @Override
  public void periodic() {
    IntakeIO.updateInputs(IntakeInputs);
    Logger.processInputs("Intake/Motors", IntakeInputs);
    
  }
}
