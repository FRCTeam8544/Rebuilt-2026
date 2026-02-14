package frc.robot.commands;

import frc.robot.subsystems.drive.Intake.*;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class IntakeCommands {

    private static final double DEADBAND = 0.1;

    private IntakeCommands() {}

    private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
        // Apply deadband
        double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
        Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

        // Square magnitude for more precise control
        linearMagnitude = linearMagnitude * linearMagnitude;

        // Return new linear velocity
        return new Pose2d(Translation2d.kZero, linearDirection)
            .transformBy(new Transform2d(linearMagnitude, 0.0, Rotation2d.kZero))
            .getTranslation();
    }

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
    public static Command openVoltageControl(Intake intake,
                                         Trigger feedTrigger,
                                         Trigger increaseVoltTrigger,
                                         Trigger decreaseVoltTrigger,
                                         Trigger increaseFeedVoltageTrigger,
                                         Trigger decreaseFeedVoltageTrigger)
    {
        return Commands.run(
        () -> {

            boolean voltIncrease = increaseVoltTrigger.getAsBoolean();
            boolean voltDecrease = decreaseVoltTrigger.getAsBoolean();
            // If and only if one button is pressed at a time
            if (voltDecrease ^ voltIncrease)
            {
                if (voltIncrease) {
                    intake.tuneIncreaseShootVoltage(); // Increase intake volt
                }
                else
                {
                    intake.tuneDecreaseShootVoltage(); // Decrease intake volt
                }
            }
            
            boolean increaseFeedVolt = increaseFeedVoltageTrigger.getAsBoolean();
            boolean decreaseFeedVolt = decreaseFeedVoltageTrigger.getAsBoolean();
            if (increaseFeedVolt ^ decreaseFeedVolt) {
                if (increaseFeedVoltageTrigger.getAsBoolean())
                {
                    intake.tuneIncreaseFeedVoltage();
                }
                else
                {
                    intake.tuneDecreaseFeedVoltage();
                }
            }

            intake.runintake(3000);
            
            if (feedTrigger.getAsBoolean())
            {
                intake.runFeedOpenLoop();
            }
            else {
                intake.runFeedOpenLoop(0.0);
            }
        },
        intake);
    }

    public static Command buttonShoot( Intake intake,
                                       Trigger shootTrigger,
                                       Trigger feedTrigger)
    {
        return Commands.run(
        () -> {
            if (shootTrigger.getAsBoolean())
            {
                intake.runintake(200);
            }
            else {
                intake.runintake(0);
            }

            if (feedTrigger.getAsBoolean())
            {
                intake.runFeed(1000);
            }
            else {
                intake.runFeed(0);
            }
        },
        intake);
    } 

    public static Command joystickVoltsShoot( Intake intake, 
                                    DoubleSupplier x_LeftSupplier, DoubleSupplier y_LeftSupplier,
                                    DoubleSupplier x_RightSupplier, DoubleSupplier y_RightSupplier ) {
        return Commands.run(
        () -> {
            
            // Get intake linear velocity
            Translation2d shootLinearVelocity =
            getLinearVelocityFromJoysticks(x_RightSupplier.getAsDouble(), y_RightSupplier.getAsDouble());              
            intake.runintakeOpenLoop(shootLinearVelocity.getY());
            
            // Get feed linear velocity
            Translation2d feedLinearVelocity =
            getLinearVelocityFromJoysticks(x_LeftSupplier.getAsDouble(), y_LeftSupplier.getAsDouble());
             
            intake.runFeedOpenLoop(feedLinearVelocity.getY());
          
        },
        intake);
    }

}