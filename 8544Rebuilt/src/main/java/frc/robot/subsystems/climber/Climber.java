package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

  public static final int climberCanId = 79; // TODO!!

  private final ClimberIOInputsAutoLogged climberInputOutputs = new ClimberIOInputsAutoLogged();
  private final ClimberIO climberIO;

  public Climber() {
    climberIO = new ClimberIOFlex(climberCanId);
  }

  public void setPosition(double rotations) {}

  public void setVoltage(double duty) {}

  @Override
  public void periodic() {
    //  climberIO.updateInputs();
  }
}
