package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.shooter.Shooter;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ShooterCommands {


    private ShooterCommands() {}

    public static Command stopMotors(Shooter shooter) {
        return Commands.run(
        () -> {
                shooter.stopMotors();
            },
            shooter);
    }

    // Use this command to tune do basic flywheel control
    public static Command openVoltageControl(Shooter shooter,
                                         Trigger increaseVoltTrigger,
                                         Trigger decreaseVoltTrigger)
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
            
            shooter.runOpenLoop();
        },
        shooter);
    }

    public static Command buttonShoot( Shooter shooter,
                                       Trigger shootTrigger,
                                       Trigger rpmAdjustDown,
                                       Trigger rpmAdjustUp)
    {
        return Commands.run(
        () -> {

            final int rpmAdjustStep = 100 / 50;

            final double shooterNominalRpm = 2720; //for tuning=2720, normal operation = 3000
           

            // These Rpms are used to tune the flywheel
            //final int shooterNominalRpm = 2720;
           // final double shooterLowNominalRpm = 337.5;
            boolean adjustUp = rpmAdjustUp.getAsBoolean();
            boolean adjustDown = rpmAdjustDown.getAsBoolean();

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

            if (shootTrigger.getAsBoolean())
            {
                shooter.runAtRpm(shooterNominalRpm);
            }
            else {
                shooter.stopMotors();
            }
        },
        shooter);
    } 

    
  /**
   * Measures the velocity feedforward constants for the shooter motors.
   *
   * <p>This command should only be used in voltage control mode.
   */
  public static Command feedforwardCharacterization(Shooter shooter) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    
    Timer timer = new Timer();

    final double FF_START_DELAY = 2.0; // Secs
    final double FF_RAMP_RATE = 0.1; // Volts/Sec

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
                  shooter.runOpenLoop(0.0);
                },
                shooter)
            .withTimeout(FF_START_DELAY),

        // Start timer
        Commands.runOnce(timer::restart),

        // Accelerate and gather data
        Commands.run(
                () -> {
                  if (!shooter.isFlywheelOverspeed()) {
                    double voltage = timer.get() * FF_RAMP_RATE;
                    shooter.runOpenLoop(voltage / Constants.kNominalVoltage);
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
