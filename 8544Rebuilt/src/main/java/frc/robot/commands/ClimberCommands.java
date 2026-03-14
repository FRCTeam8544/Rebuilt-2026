package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.climber.*;


public class ClimberCommands {


  private ClimberCommands() {}


  public static Command stopMotors(Climber climber) {
    return Commands.run(
        () -> {
        //  climber.holdArmPosition();
          climber.runOpenLoop(0);
        },
        climber);
  }


  public static Command closedPositionControl(
      Climber climber,
      Trigger extendTrigger,
      Trigger retractTrigger
      ) {
    return Commands.run(
        () -> {
          boolean extendPosition = extendTrigger.getAsBoolean();
          boolean retractPosition = retractTrigger.getAsBoolean();
          // If and only if one button is pressed at a time
          if (retractPosition ^ extendPosition) {
            if (extendPosition) {
              climber.runHookToPosition(0.7); // Set Extend Position .7
            } else {
              climber.runHookToPosition(0.3); // Set Retract Position .3
            }
           } else { climber.holdHookPosition();

           }
          

        },
        climber);
  }

  public static Command openVoltageControl(
      Climber climber,
      Trigger extendTrigger,
      Trigger retractTrigger
      ) {
    return Commands.run(
        () -> {
          boolean extendPosition = extendTrigger.getAsBoolean();
          boolean retractPosition = retractTrigger.getAsBoolean();
          double climberDuty = 0.3;
          // If and only if one button is pressed at a time
          if (retractPosition ^ extendPosition) {
            if (extendPosition) {
              climber.runOpenLoop(climberDuty); // Extend
            } else {
              climber.runOpenLoop(-climberDuty); // Retract
            }
           }
           else {
              climber.runOpenLoop(0);
           }
        },
        climber);
  }

}