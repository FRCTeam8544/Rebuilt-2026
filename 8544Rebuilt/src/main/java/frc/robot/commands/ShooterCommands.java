package frc.robot.commands;

import frc.robot.subsystems.shooter.Shooter;

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

public class ShooterCommands {

    private static final double DEADBAND = 0.1;

    private ShooterCommands() {}

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

    public static Command stopMotors(Shooter shooter) {
        return Commands.run(
        () -> {
                shooter.stopOpenLoop();
            },
            shooter);
    }

    // Use this command to tune Ks by increasing voltage until the flywheel
    // begins to slightly turn, then back off a bit.
    // This term will be used in the PID with feedforward control later.
    public static Command openVoltageControl(Shooter shooter,
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
                    shooter.tuneIncreaseShootVoltage(); // Increase shooter volt
                }
                else
                {
                    shooter.tuneDecreaseShootVoltage(); // Decrease shooter volt
                }
            }
            
            boolean increaseFeedVolt = increaseFeedVoltageTrigger.getAsBoolean();
            boolean decreaseFeedVolt = decreaseFeedVoltageTrigger.getAsBoolean();
            if (increaseFeedVolt ^ decreaseFeedVolt) {
                if (increaseFeedVoltageTrigger.getAsBoolean())
                {
                    shooter.tuneIncreaseFeedVoltage();
                }
                else
                {
                    shooter.tuneDecreaseFeedVoltage();
                }
            }

            shooter.runShooter(3000);
            
            if (feedTrigger.getAsBoolean())
            {
                shooter.runFeedOpenLoop();
            }
            else {
                shooter.runFeedOpenLoop(0.0);
            }
        },
        shooter);
    }

    public static Command buttonShoot( Shooter shooter,
                                       Trigger shootTrigger,
                                       Trigger feedTrigger,
                                       Trigger rpmAdjustDown,
                                       Trigger rpmAdjustUp,
                                       Trigger resetShooterDefaults)
    {
        return Commands.run(
        () -> {

            final int rpmAdjustStep = 5;
            final int shooterNominalRpm = 3000;
            boolean adjustUp = rpmAdjustUp.getAsBoolean();
            boolean adjustDown = rpmAdjustDown.getAsBoolean();

            if (!resetShooterDefaults.getAsBoolean())
            {
                if (adjustUp ^ adjustDown)
                {
                    if (adjustUp) {
                        shooter.shooterRpmAdjust(rpmAdjustStep);
                    }
                    else
                    {
                        shooter.shooterRpmAdjust(-rpmAdjustStep);
                    }
                }
            }
            else {
                shooter.resetDefaultRpms();
            }

            if (shootTrigger.getAsBoolean())
            {
                shooter.runShooter(shooterNominalRpm);
            }
            else {
                shooter.stopShooter();
            }

            if (feedTrigger.getAsBoolean())
            {
                shooter.runFeedOpenLoop(0.6);
            }
            else {
                shooter.runFeedOpenLoop(0);
            }
        },
        shooter);
    } 

    public static Command joystickVoltsShoot( Shooter shooter, 
                                    DoubleSupplier x_LeftSupplier, DoubleSupplier y_LeftSupplier,
                                    DoubleSupplier x_RightSupplier, DoubleSupplier y_RightSupplier ) {
        return Commands.run(
        () -> {
            
            // Get shooter linear velocity
            Translation2d shootLinearVelocity =
            getLinearVelocityFromJoysticks(x_RightSupplier.getAsDouble(), y_RightSupplier.getAsDouble());              
            shooter.runShooterOpenLoop(shootLinearVelocity.getY());
            
            // Get feed linear velocity
            Translation2d feedLinearVelocity =
            getLinearVelocityFromJoysticks(x_LeftSupplier.getAsDouble(), y_LeftSupplier.getAsDouble());
             
            shooter.runFeedOpenLoop(feedLinearVelocity.getY());
          
        },
        shooter);
    }

}
