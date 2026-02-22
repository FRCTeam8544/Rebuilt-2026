package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Arm.*;


public class ArmCommands {


  private ArmCommands() {}

  public static Command openLoopControl(
    Arm arm,
    Trigger armInTrigger,
    Trigger armOutTrigger,
    Trigger intakeTrigger,
    Trigger expelTrigger) {
      return Commands.run(
        () -> {

          final double armExtendDuty = 0.6;
          final double armRetractDuty = -0.8;
          boolean extendPosition = armOutTrigger.getAsBoolean();
          boolean retractPosition = armInTrigger.getAsBoolean();
          // If and only if one button is pressed at a time
          if (retractPosition ^ extendPosition) {
            if (extendPosition) {
              arm.runArmOpenLoop(armExtendDuty); // Set Extend Position
            } else {
              arm.runArmOpenLoop(armRetractDuty); // Set Retract Position
            }
          }
          else { 
            arm.runArmOpenLoop(0.0);
          }});


      }

  public static Command stopMotors(Arm arm) {
    return Commands.run(
        () -> {
          arm.stopOpenLoop();
         
        },
        arm);
  }


  public static Command closedPositionControl(
      Arm arm,
      Trigger extendTrigger,
      Trigger retractTrigger,
      Trigger intakeTrigger,
      Trigger expelTrigger
      ) {
    return Commands.run(
        () -> {
          boolean extendPosition = extendTrigger.getAsBoolean();
          boolean retractPosition = retractTrigger.getAsBoolean();
          // If and only if one button is pressed at a time
          if (retractPosition ^ extendPosition) {
            if (extendPosition) {
              arm.runArm(0.8); // Set Extend Position
            } else {
              arm.runArm(0.2); // Set Retract Position
            }
           } else { arm.holdArmPosition();

           }

  });}}

  