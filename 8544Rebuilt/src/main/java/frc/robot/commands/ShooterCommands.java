package frc.robot.commands;

import frc.robot.subsystems.shooter.Shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

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

    public static Command joystickShoot( Shooter shooter, DoubleSupplier xSupplier, DoubleSupplier ySupplier ) {
        return Commands.run(
        () -> {
          // Get linear velocity
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());
          
          shooter.runVoltage(linearVelocity.getY());
          
        },
        shooter);
    }

}
