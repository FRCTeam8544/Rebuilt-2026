package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Intake.*;


public class IntakeCommands {


  private IntakeCommands() {}

  public static Command openLoopControl(
    Intake intake,
    Trigger armInTrigger,
    Trigger armOutTrigger,
    Trigger intakeTrigger,
    Trigger expelTrigger) {
      return Commands.run(
        () -> {

          final double armExtendDuty = 0.3;
          final double armRetractDuty = -0.4;
          boolean extendPosition = armOutTrigger.getAsBoolean();
          boolean retractPosition = armInTrigger.getAsBoolean();
          // If and only if one button is pressed at a time
          if (retractPosition ^ extendPosition) {
            if (extendPosition) {
              intake.runArmOpenLoop(armExtendDuty); // Set Extend Position
            } else {
              intake.runArmOpenLoop(armRetractDuty); // Set Retract Position
            }
          }
          else { 
            intake.runArmOpenLoop(0.0);
          }

          final double intakeFeedDuty = 0.4;
          boolean intakeFuel = intakeTrigger.getAsBoolean();
          boolean expelFuel = expelTrigger.getAsBoolean();
          if (intakeFuel ^ expelFuel) {
            if (intakeFuel) {
              intake.runFeedOpenLoop(intakeFeedDuty);
            }
            else {
              intake.runFeedOpenLoop(-intakeFeedDuty);
            }
          }
          else {
            intake.runFeedOpenLoop(0.0);
          }

        },
        intake);
      }

  public static Command stopMotors(Intake intake) {
    return Commands.run(
        () -> {
          intake.stopOpenLoop();
          intake.runIntakeFeed(0);
        },
        intake);
  }


  public static Command closedPositionControl(
      Intake intake,
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
              intake.runIntakeArm(0.8); // Set Extend Position
            } else {
              intake.runIntakeArm(0.2); // Set Retract Position
            }
           } else { intake.holdArmPosition();

           }
          
          boolean inCmd = intakeTrigger.getAsBoolean();
          boolean outCmd = expelTrigger.getAsBoolean();
          if (inCmd ^ outCmd) {
            if (inCmd) {
              intake.runIntakeFeed(400);
            }
            else {
              intake.runIntakeFeed(-400);
            }
          }
          else {
            intake.runIntakeFeed(0);
          }

        },
        intake);
  }
  }

  

