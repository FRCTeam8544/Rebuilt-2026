package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.Feeder.*;
import frc.robot.subsystems.vision.*;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class BattlecryCommands {


    private BattlecryCommands() {}

    // Use this command to tune do basic flywheel control
    public static Command scream(Shooter shooter,
                                 Feeder feeder)
    {
        SlewRateLimiter limiter = new SlewRateLimiter(3500);
        return Commands.run(

        () -> {
            // Lock out the feeder while screaming

            shooter.runAtRpm(limiter.calculate(Shooter.Flywheel.kMaxShooterRPM));
            
        },
        shooter).finallyDo( () -> { shooter.runOpenLoop(0); limiter.reset(0); });
    }



}
