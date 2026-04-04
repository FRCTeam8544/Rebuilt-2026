package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Intake.*;


public class IntakeCommands {


  private IntakeCommands() {}

  public static Command stopMotors(Intake intake) {
    return Commands.run(
        () -> {
          intake.stopMotors();
        },
        intake);
  }

  public static Command runAtDuty(Intake intake, double intakeDuty) {
    return Commands.run(
      () -> {
        intake.runOpenLoop(intakeDuty);
      },
      intake).withName("runAtDuty");
  }

  public static Command openLoopControl(
    Intake intake,
    Trigger intakeTrigger,
    Trigger expelTrigger) {
      return Commands.run(
        () -> {

          final double intakeFeedDuty = -0.9;

          boolean intakeFuel = intakeTrigger.getAsBoolean();
          boolean expelFuel = expelTrigger.getAsBoolean();
          if (intakeFuel ^ expelFuel) {
            if (intakeFuel) {
              intake.runOpenLoop(intakeFeedDuty);
            } else {
              intake.runOpenLoop(-intakeFeedDuty);
            }
          }
          else {
            intake.runOpenLoop(0.0);
          }

        },
        intake).withName("openLoopControl");
      }

  public static Command intakeFuel(Intake intake) {
    final double intakeFeedDuty = -0.9;
    return Commands.run(
      () -> {
        intake.runOpenLoop(intakeFeedDuty);
      },
      intake).withName("intakeFuel");
  }

  public static Command closeLoopControl(
      Intake intake,
      Trigger intakeTrigger,
      Trigger expelTrigger) {
    return Commands.run(
        () -> {

          final double intakeSpeedRpm = 1120;
          boolean inPressed = intakeTrigger.getAsBoolean();
          boolean outPressed = expelTrigger.getAsBoolean();
          if (inPressed ^ outPressed) {
            if (inPressed) {
              intake.runAtRpm(intakeSpeedRpm);
            } else {
              intake.runAtRpm(-intakeSpeedRpm);
            }
          }
          else {
            intake.stopMotors();
          }

        },
        intake).withName("closeLoopControl");
  }

  public static Command oneButtonControl(
    Intake intake,
    Boolean onebuttonTrigger
    ) {
    return Commands.run(
      () -> {

        final double intakeFeedDuty = 0.8; //full speed (too fast)
        boolean oneButtonPressed = onebuttonTrigger;

        if (oneButtonPressed) {
          intake.runOpenLoop(-intakeFeedDuty);
        } else {
          intake.stopMotors();
        }
      },
      intake).withName("oneButtonControl");
  }

}

  

