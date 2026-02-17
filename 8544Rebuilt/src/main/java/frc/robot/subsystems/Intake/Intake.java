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

  public final IntakeIO IntakeIO;
  public final IntakeIOInputsAutoLogged IntakeInputs = new IntakeIOInputsAutoLogged();

  private final IntakeFeedIO intakeFeedIO;
  private final IntakeFeedIOInputsAutoLogged intakeFeedInputs = new IntakeFeedIOInputsAutoLogged();

  public Intake() {
    this.IntakeIO = new IntakeIOMax(armCanId);
    this.intakeFeedIO = new IntakeFeedIOMax(feedCanId);
  }

  public void runIntake(double rotations) {
    IntakeInputs.positionSetPoint = rotations;
    IntakeIO.setPosition(IntakeInputs.positionSetPoint);
  } 

  public void runIntakeFeed(double rpm) {
    intakeFeedInputs.velocitySetPoint = rpm;
    intakeFeedIO.setVelocity(intakeFeedInputs.velocitySetPoint);
  }
  
  public void stopFeed() {
    intakeFeedInputs.velocitySetPoint = 0;
    intakeFeedIO.setVelocity(intakeFeedInputs.velocitySetPoint);
  }

  public void holdPosition() {
    IntakeInputs.positionSetPoint = IntakeInputs.position;
    IntakeIO.setPosition(IntakeInputs.position);
  }

  public void stopOpenLoop() {
    IntakeIO.setVoltage(0);
  }

  @Override
  public void periodic() {
    IntakeIO.updateInputs(IntakeInputs);
    intakeFeedIO.updateInputs(intakeFeedInputs);
    Logger.processInputs("Intake/Motors", IntakeInputs);
    Logger.processInputs("Intake/FeedMotor", intakeFeedInputs);
    
  }
}
