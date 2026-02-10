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

    public static Command buttonShoot( Shooter shooter,
                                       Trigger shootTrigger,
                                       Trigger feedTrigger)
    {
        return Commands.run(
        () -> {
            if (shootTrigger.getAsBoolean())
            {
                shooter.runShooter(200);
            }
            else {
                shooter.runShooter(0);
            }

            if (feedTrigger.getAsBoolean())
            {
                shooter.runFeed(1000);
            }
            else {
                shooter.runFeed(0);
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
