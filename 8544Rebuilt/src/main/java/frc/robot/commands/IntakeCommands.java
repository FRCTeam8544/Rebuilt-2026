package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Intake.*;
//import frc.robot.subsystems.Intake.IntakeIO.IntakeIOInputs;
import frc.robot.subsystems.Intake.IntakeIO.IntakeIOInputs;

import java.util.function.DoubleSupplier;

public class IntakeCommands {


  private IntakeCommands() {}

  

  public static Command stopMotors(Intake intake) {
    return Commands.run(
        () -> {
          intake.stopOpenLoop();
        },
        intake);
  }

  // Use this command to tune Ks by increasing voltage until the flywheel
  // begins to slightly turn, then back off a bit.
  // This term will be used in the PID with feedforward control later.
  public static Command openVoltageControl(
      Intake intake,
      Trigger extendTrigger,
      Trigger retractTrigger,
      Trigger increasepositionTrigger,
      Trigger decreasepositionTrigger
    //  Trigger decreaseFeedVoltageTrigger
      ) {
    return Commands.run(
        () -> {
          boolean positionIncrease = increasepositionTrigger.getAsBoolean();
          boolean positionDecrease = decreasepositionTrigger.getAsBoolean();
          // If and only if one button is pressed at a time
          if (positionDecrease ^ positionIncrease) {
            if (positionIncrease) {
              intake.runIntakeOpenLoop(true); // Increase intake volt
            } else {
              intake.runIntakeOpenLoop(false); // Decrease intake volt
            }
          }
          else {
            intake.stopOpenLoop();
          }
 
          boolean voltageIncrease = increasepositionTrigger.getAsBoolean();
          boolean voltageDecrease = decreasepositionTrigger.getAsBoolean();
          // If and only if one button is pressed at a time
          if (voltageDecrease ^ voltageIncrease) {
            if (voltageIncrease) {
              intake.tuneIncreaseVoltage(); // Increase intake volt
            } else {
              intake.tuneDecreaseVoltage(); // Decrease intake volt
            }
          }
        },
        intake);
  }
  public static Command closedPositionControl(
      Intake intake,
      Trigger extendTrigger,
      Trigger retractTrigger
      //Trigger increasepositionTrigger,
      //Trigger decreasepositionTrigger
    //  Trigger decreaseFeedVoltageTrigger
      ) {
    return Commands.run(
        () -> {
          boolean extendPosition = extendTrigger.getAsBoolean();
          boolean retractPosition = retractTrigger.getAsBoolean();
          // If and only if one button is pressed at a time
          if (retractPosition ^ extendPosition) {
            if (extendPosition) {
              intake.runIntake(0.5); // Increase intake volt
            } else {
              intake.runIntake(0); // Decrease intake volt
            }
           } else { intake.runIntake(IntakeIOMax.realPosition);

           }
            

        },
        intake);
  }
/*   for position ctrl
  public static Command buttonShoot(Intake intake, Trigger extendTrigger, Trigger retractTrigger) {
    return Commands.run(
        () -> {
          if (extendTrigger.getAsBoolean()) {
            intake.runintake(200);
          } else {
            intake.runintake(0);
          }

          if (retractTrigger.getAsBoolean()) {
            intake.runFeed(1000);
          } else {
            intake.runFeed(0);
          }
        },
        intake);
        */
  }

  

