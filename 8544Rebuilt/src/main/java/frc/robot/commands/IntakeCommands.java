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

          final double intakeFeedDuty = 0.7;

          boolean intakeFuel = intakeTrigger.getAsBoolean();
          boolean expelFuel = expelTrigger.getAsBoolean();
          boolean active = intakeFuel ^ expelFuel;
          if (active) {
            leds.setMechanicalState(Leds.MechanicalState.INTAKING);
            intake.runOpenLoop(intakeFuel ? intakeFeedDuty : -intakeFeedDuty);
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
          boolean active = inPressed ^ outPressed;

          if (active) {
            leds.setMechanicalState(Leds.MechanicalState.INTAKING);
            intake.runAtRpm(inPressed ? intakeSpeedRpm : -intakeSpeedRpm);
          }
          else {
            leds.setMechanicalState(Leds.MechanicalState.NONE);
            intake.stopMotors();
          }

        },
        intake);
  }

  }

  

