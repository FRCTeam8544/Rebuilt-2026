package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Intake.*;
import frc.robot.subsystems.leds.Leds;



public class IntakeCommands {


  private IntakeCommands() {}

  public static Command stopMotors(Intake intake) {
    return Commands.run(
        () -> {
          intake.stopMotors();
        },
        intake);
  }

  public static Command openLoopControl(
    Intake intake,
    Trigger intakeTrigger,
    Trigger expelTrigger,
    Leds leds) {
      return Commands.run(
        () -> {

          final double intakeFeedDuty = 0.9;

          boolean intakeFuel = intakeTrigger.getAsBoolean();
          boolean expelFuel = expelTrigger.getAsBoolean();
          if (intakeFuel ^ expelFuel) {
            if (intakeFuel) {
              leds.setMechanicalState(Leds.MechanicalState.INTAKING);
              intake.runOpenLoop(intakeFeedDuty);
            } else {
              leds.setMechanicalState(Leds.MechanicalState.NONE);
              intake.runOpenLoop(-intakeFeedDuty);
            }
          }
          else {
            leds.setMechanicalState(Leds.MechanicalState.NONE);
            intake.runOpenLoop(0.0);
          }

        },
        intake);
      }


  public static Command closedPositionControl(
      Intake intake,
      Trigger intakeTrigger,
      Trigger expelTrigger,
      Leds leds) {
    return Commands.run(
        () -> {

          final double intakeSpeedRpm = 420;
          boolean inPressed = intakeTrigger.getAsBoolean();
          boolean outPressed = expelTrigger.getAsBoolean();
          if (inPressed ^ outPressed) {
            if (inPressed) {
              leds.setMechanicalState(Leds.MechanicalState.INTAKING);
              intake.runAtRpm(intakeSpeedRpm);
            } else {
              leds.setMechanicalState(Leds.MechanicalState.NONE);
              intake.runAtRpm(-intakeSpeedRpm);
            }
          }
          else {
            leds.setMechanicalState(Leds.MechanicalState.NONE);
            intake.stopMotors();
          }

        },
        intake);
  }

  }

  

