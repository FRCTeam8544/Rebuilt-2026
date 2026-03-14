package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Arm.*;

public class ArmCommands {

  private ArmCommands() {}

  public static Command openLoopLimitedVoltage(
    Arm arm,
    Trigger armInTrigger,
    Trigger armOutTrigger) {
      return Commands.run(
        () -> {
          final double armExtendDuty = -0.5;
          final double armRetractDuty = 0.5;
          boolean extendPressed = armOutTrigger.getAsBoolean();
          boolean retractPressed = armInTrigger.getAsBoolean();
          // If and only if one button is pressed at a time
          if (retractPressed ^ extendPressed) {
            if (extendPressed) {
              arm.runOpenLoopLimited(armExtendDuty); // Set Extend Position
            } else {
              arm.runOpenLoopLimited(armRetractDuty); // Set Retract Position
            }
          }
          else { 
            arm.stopOpenLoop();
          }
        },
        arm );
    }

  public static Command openLoopControl(
    Arm arm,
    Trigger armInTrigger,
    Trigger armOutTrigger) {
      return Commands.run(
        () -> {

          final double armExtendDuty = -0.3;
          final double armRetractDuty = 0.3;
          boolean extendPressed = armOutTrigger.getAsBoolean();
          boolean retractPressed = armInTrigger.getAsBoolean();
          // If and only if one button is pressed at a time
          if (retractPressed ^ extendPressed) {
            if (extendPressed) {
              arm.runOpenLoop(armExtendDuty); // Set Extend Position
            } else {
              arm.runOpenLoop(armRetractDuty); // Set Retract Position
            }
          }
          else { 
            arm.stopOpenLoop();
          }

        },
        arm);
  }

  public static Command stopMotors(Arm arm) {
    return Commands.run(
        () -> {
          arm.stopMotors();
        },
        arm);
  }

  public static Command closedPositionControl(
      Arm arm,
      Trigger extendTrigger,
      Trigger retractTrigger
      ) {
    return Commands.run(
        () -> {
          boolean extendPressed = extendTrigger.getAsBoolean();
          boolean retractPressed = retractTrigger.getAsBoolean();

          final double extendPosition = 0.78; // 0.8;
          final double retractPosition = 0.037; //0.2;

          // If and only if one button is pressed at a time
          if (retractPressed ^ extendPressed) {
            if (extendPressed) {
              arm.runToPosition(extendPosition); // Set Extend Position
            } else {
              arm.runToPosition(retractPosition); // Set Retract Position
            }
          } else { 
            arm.holdPosition();
          }

          },
          arm);
  }


    public static Command oneButtonControl(
      Arm arm,
      Boolean onebuttonTrigger
      ) {
    return Commands.run(
        () -> {
          boolean oneButton = onebuttonTrigger;

          final double extendPosition = 0.78; // 0.8;
          final double retractPosition = 0.037; //0.2;

       
            if (oneButton) {
              arm.runToPosition(extendPosition); // Set Extend Position
            } else {
              arm.runToPosition(retractPosition); // Set Retract Position
            }
       //   } else { 
         //   arm.holdPosition();
        
        
          

      },
          arm);
  }
}


  