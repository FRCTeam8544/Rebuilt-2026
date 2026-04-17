package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Arm.*;

public class ArmCommands {

  private ArmCommands() {}

  public static Command openLoopControl(
    Arm arm,
    Trigger armInTrigger,
    Trigger armOutTrigger) {
      return Commands.run(
        () -> {

          final double armExtendDuty = -0.9;
          final double armRetractDuty = 0.9;
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
        arm).withName("openLoopControl");
  }

  public static Command stopMotors(Arm arm) {
    return Commands.run(
      () -> {
        arm.stopMotors();
      },
      arm).withName("stopMotors");
  }

  public static Command runToPosition( Arm arm, double armPosition) {
    return Commands.run (
      () -> {
        arm.runToPosition(armPosition);
      },
      arm).withName("runToPosition");
  }

  public static Command runToVoltage( Arm arm, double armVoltage) {
    return Commands.run (
      () -> {            // Duty, not voltage!
        arm.runOpenLoop(armVoltage);
      },
      arm).withName("runToVoltage");
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
          arm).withName("closedPositionControl");
  }

  public static Command oneButtonControl(
      Arm arm,
      Boolean onebuttonTrigger
      ) {

        return Commands.run(


      
    
        () -> {
          boolean oneButton = onebuttonTrigger;

          final double extendPosition = 0.78;
          final double retractPosition = 0.037;

            if (oneButton) {
              arm.runOpenLoop(0.9); // Set Extend Position // arm.runToPosition(extendPosition);
            } else {
              arm.runOpenLoop(-0.9); // Set Retract Position
            }
            
      },
      arm).withName("oneButtonControl");
  }
  
  // Use PID control to hold position
  public static Command holdPosition( Arm arm ) {
    return Commands.startRun(
      () -> { arm.runToPosition(arm.armPositionSupplier.getAsDouble());},
      () -> { arm.holdPosition(); },

      arm
    );
  }

  public static Command deployHopper( Arm arm ) {
    double deployTimelimitSeconds = 2;
    return Commands.runEnd(
      () -> arm.runOpenLoop(0.7),
      () -> arm.stopOpenLoop(),
      arm).withName("deployHopper")
          .withTimeout(deployTimelimitSeconds);
  }

  public static Command retractHopper( Arm arm ) {
    final double retractTimeLimitSeconds = 3.0;
    return Commands.runEnd(
      () -> arm.runOpenLoop(-0.7),
      () -> arm.stopOpenLoop(),
      arm).withName("retractHopper")
          .withTimeout(retractTimeLimitSeconds);
  }


}


  