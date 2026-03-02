package frc.robot.commands;

import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.Feeder.*;  //TODO move to new FeederCommands.java
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ShooterCommands {

    private static final double FF_START_DELAY = 2.0; // Secs
    private static final double FF_RAMP_RATE = 0.1; // Volts/Sec

    private ShooterCommands() {}

    public static Command stopMotors(Shooter shooter) {
        return Commands.run(
        () -> {
                shooter.stopShooter();
            },
            shooter);
    }

    // Use this command to tune Ks by increasing voltage until the flywheel
    // begins to slightly turn, then back off a bit.
    // This term will be used in the PID with feedforward control later.
    public static Command openVoltageControl(Shooter shooter,  
                                         Trigger flywheelTrigger,
                                         Trigger decreaseVoltTrigger,
                                         Trigger increaseVoltTrigger
                                         )
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
            
            if (flywheelTrigger.getAsBoolean()) {
                shooter.runOpenLoop();
            }
            else {
                shooter.stopShooter();
            }
            
        },
        shooter);
    }

    // Replace with kraken impplementation
 /*   public static Command buttonShoot( Shooter shooter,
                                       Trigger flywheelTrigger,
                                       Trigger rpmAdjustDown,
                                       Trigger rpmAdjustUp)
    {
        return Commands.run(
        () -> {

            final int rpmAdjustStep = 5;
            final int shooterNominalRpm = 3000;
            boolean adjustUp = rpmAdjustUp.getAsBoolean();
            boolean adjustDown = rpmAdjustDown.getAsBoolean();

            if (adjustUp ^ adjustDown)
            {
                if (adjustUp) {
                    //shooter.shooterRpmAdjust(rpmAdjustStep);
                    shooter.tuneIncreaseShootVoltage();
                }
                else
                {
                    //shooter.shooterRpmAdjust(-rpmAdjustStep);
                    shooter.tuneDecreaseShootVoltage();
                }
            }

            shooter.runOpenLoop();  //may remove

            if (flywheelTrigger.getAsBoolean())
            {
                shooter.runOpenLoop();
            }
            else {
                shooter.stopShooter();
            }




        },
        shooter);
    } */
    
  /**
   * Measures the velocity feedforward constants for the shooter motors.
   *
   * <p>This command should only be used in voltage control mode.
   */
  public static Command feedforwardCharacterization(Shooter shooter) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    
    Timer timer = new Timer();

    return Commands.sequence(
        // Reset data
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
            }),

        // Warmup
        Commands.run(
                () -> {
                  shooter.runShooterOpenLoop(0.0);
                },
                shooter)
            .withTimeout(FF_START_DELAY),

        // Start timer
        Commands.runOnce(timer::restart),

        // Accelerate and gather data
        Commands.run(
                () -> {
                  if (shooter.getFlywheelVelocityRPM() < Shooter.Flywheel.kMaxShooterRPM) {
                    double voltage = timer.get() * FF_RAMP_RATE;
                    shooter.runShooterOpenLoop(voltage / 12.0);
                    velocitySamples.add(shooter.getShooterFFCharacterizationVelocity());
                    voltageSamples.add(voltage);
                  }
                },
                shooter)

            // When cancelled, calculate and print results
            .finallyDo(
                () -> {
                  int n = velocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                  }
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println("********** Shooter FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                }));
  }


}
