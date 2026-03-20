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

          final double armExtendDuty = -0.8;
          final double armRetractDuty = 0.8;
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
      () -> {
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
        Timer timer = new Timer();
        return Commands.startRun(
() -> {

timer.restart();


}

      
    ,
        () -> {
          boolean oneButton = onebuttonTrigger;

          final double extendPosition = 0.78;
          final double retractPosition = 0.037;
if (timer.get()<0.5) {
            if (oneButton) {
              arm.runOpenLoop(0.3); // Set Extend Position // arm.runToPosition(extendPosition);
            } else {
              arm.runOpenLoop(-0.3); // Set Retract Position
            }
            }
      },
      arm).withName("oneButtonControl");
  }
  
  public static Command deployHopper( Arm arm ) {

    final double extendPosition = ArmIO.kNominalDeployPosition;
    double deployTimelimit = 2; // seconds
    return Commands.run(
    () -> {
      arm.runToPosition(extendPosition);
    },
    arm).withName("deployHopper")
        .until(arm.armDeployedSupplier)
        .withTimeout(deployTimelimit);
  }

  public static Command retractHopper( Arm arm ) {
    final double retractPosition = ArmIO.kNominalStowPosition;
    final double retractTimeLimit = 3.0;
    return Commands.run(
      () -> {
        arm.runToPosition(retractPosition);
      },
      arm).withName("retractHopper")
          .until(arm.armRetractedSupplier)
          .withTimeout(retractTimeLimit);
  }


}


  