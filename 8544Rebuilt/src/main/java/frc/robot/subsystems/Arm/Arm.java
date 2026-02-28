package frc.robot.subsystems.Arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm.ArmIO;
import frc.robot.subsystems.Arm.ArmIOMax;



import static edu.wpi.first.units.Units.Rotations;

import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {


  public static final double kMaxIntakeRPM = 2000; //100:1 gearbox


  public static final int armCanId = 27;
  public final ArmIO ArmIO;
  public final ArmIOInputsAutoLogged ArmInputs = new ArmIOInputsAutoLogged();


  public Arm() {
    this.ArmIO = new ArmIOMax(armCanId);

  }

  public void runArmOpenLoop(double duty) {
    
   // final double forwardLimit = 0.95; // Zero is "straight up" 870
   // final double backwardLimit = 0.07; // Stow position

    double adjustedDuty = duty;
    adjustedDuty = Math.min(adjustedDuty, 1.0);
    adjustedDuty = Math.max(adjustedDuty, -1.0);

    ArmInputs.voltageSetPoint = adjustedDuty * Constants.Neo.nominalVoltage;
    ArmInputs.positionSetPoint = 0.0;

      ArmIO.setVoltage(ArmInputs.voltageSetPoint);
   
  }



  public void runArm(double rotations) {
    ArmInputs.positionSetPoint = rotations;
    ArmInputs.voltageSetPoint = 0.0;
    ArmIO.setPosition(ArmInputs.positionSetPoint);
  } 


  public void holdArmPosition() {
    ArmInputs.positionSetPoint = ArmInputs.position;
    ArmIO.setPosition(ArmInputs.position);
  }

  public void stopOpenLoop() {
    ArmIO.setVoltage(0);
  }

  @Override
  public void periodic() {
    ArmIO.updateInputs(ArmInputs);
    Logger.processInputs("Intake/Arm",ArmInputs);
    
  }
}