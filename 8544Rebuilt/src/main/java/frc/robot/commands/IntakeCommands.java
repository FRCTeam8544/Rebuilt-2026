package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Intake.*;


public class IntakeCommands {


  private IntakeCommands() {}

  public static Command feedVelocityControl(
    Intake intake,
    Trigger intakeTrigger,
    Trigger expelTrigger) {
      return Commands.run(
        () -> {
          
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
              intake.runIntake(0.8); // Set Extend Position
            } else {
              intake.runIntake(0.2); // Set Retract Position
            }
           } else { intake.holdPosition();

           }
          
          boolean inCmd = intakeTrigger.getAsBoolean();
          boolean outCmd = expelTrigger.getAsBoolean();
          if (inCmd ^ outCmd) {
            if (inCmd) {
              intake.runIntakeFeed(200);
            }
            else {
              intake.runIntakeFeed(-200);
            }
          }
          else {
            intake.runIntakeFeed(0);
          }

        },
        intake);
  }
  }

  

