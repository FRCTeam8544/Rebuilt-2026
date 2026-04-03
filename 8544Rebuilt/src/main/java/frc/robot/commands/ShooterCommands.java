package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.shooter.Shooter;
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

    public static Command runFlywheel(Shooter shooter,
                                      DoubleSupplier rpmSetPoint
                                      )
    {
        return Commands.run(
            () -> {
                 shooter.runAtRpm(rpmSetPoint.getAsDouble());
            },
            shooter);
    }

    public static Command buttonShoot( Shooter shooter,
                                        DoubleSupplier rpmByDistanceSupplier,
                                        BooleanSupplier AutoToggleSupplier,
                                       Trigger rpmAdjustDown,
                                       Trigger rpmAdjustUp)
    {
        final double maxRpmAdjustPerSecond = 3000;
        SlewRateLimiter rpmAdjustLimiter = new SlewRateLimiter(maxRpmAdjustPerSecond);
        
        return Commands.startRun(
        // Run once to reset the rate limiter to the current RPM when command starts
        () -> { rpmAdjustLimiter.reset( shooter.flywheelRpmSupplier.getAsDouble()); },
        // Run until interrupted
        () -> {
            final int rpmAdjustStep = 100 / 50;
             
            final double shooterNominalRpm = 3000;
            double shooterRpmAuto = rpmByDistanceSupplier.getAsDouble();

            // These Rpms are used to tune the flywheel
            //final int shooterNominalRpm = 2720;
           // final double shooterLowNominalRpm = 337.5;
            boolean adjustUp = rpmAdjustUp.getAsBoolean();
            boolean adjustDown = rpmAdjustDown.getAsBoolean();
            boolean FlywheelAutoRPMLocal = AutoToggleSupplier.getAsBoolean();

            // TODO this adjust logic really should just be inside of the command instead of shooter subsystem
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

            double shooterRpmSetpoint = shooterNominalRpm;

            if(FlywheelAutoRPMLocal) { 
                shooterRpmSetpoint = shooterRpmAuto;
            }

            // Use rpmAdjustLimiter to slow down massive swings in the RPM setpoints
            // No longer commands full shooting RPM (3k!) immediately. The setpoint
            // will increase over time until it gets to the desired RPM setpoint.
            // However the limiter should never be set so tight that it limits the
            // small, 100s of RPM, adjustments made for auto shoot.
            shooter.runAtRpm(rpmAdjustLimiter.calculate(shooterRpmSetpoint));
            
        },
        shooter).finallyDo( () -> { shooter.resetShooterDefaultRpm(); 
        }); // reset rpm setpoint of shooter
    
    } 

  public static Command gentleStopFlywheel(Shooter shooter)
  {
    double breakDisableRpm = 500; // Below this RPM friction will take the wheel to zero
    double maxBrakeVoltage = -2.0; // Must be negative: Maximum voltage to apply to stop the flywheel over time
    double breakApplyTimeSeconds = 1; // Time to full breaking power
    double breakTimeoutSeconds = 10.0; // Time limit to break the wheel
    // Limit voltage rate change over time aka gentle breaking
    SlewRateLimiter voltageLimiter = new SlewRateLimiter(
                                        Math.abs(maxBrakeVoltage) / breakApplyTimeSeconds); 
    return Commands.sequence(
        // "Stop" Pid control
        Commands.runOnce(
        () -> {
              shooter.stopMotors();
              voltageLimiter.reset(0); // Start braking at zero volts
            }),
        Commands.run(
        () -> {
            if (shooter.flywheelRpmSupplier.getAsDouble() > breakDisableRpm) {
                double duty = voltageLimiter.calculate(maxBrakeVoltage) / Constants.kNominalVoltage;
                shooter.runOpenLoop(duty);
            }
            else {
                shooter.stopMotors();
            }
        },
        shooter).finallyDo( () -> { shooter.stopMotors(); }) // Ensure flywheel stops under all conditions
    ).withTimeout(breakTimeoutSeconds).withName("Braking");
  }

  // Default, not running the flywheel state
  public static Command idleFlywheel(Shooter shooter)
  {
    return Commands.run(
        () -> {
            shooter.stopMotors();
        },
        shooter).withName("Idle");
  }

  // If the FMS is active, hub active, and robot is in the scoring zone prepare the flywheel
  public static Command autoIdleFlywheel(Shooter shooter, 
                                         BooleanSupplier hubActive,
                                         BooleanSupplier inScoringZone,
                                         BooleanSupplier fmsAvailable)
  {
    final double activeIdleRpm = 2000;
    final double maxRpmAdjustPerSecond = 3000;
    SlewRateLimiter rpmAdjustLimiter = new SlewRateLimiter(maxRpmAdjustPerSecond);
        
    return Commands.startRun(
        () -> {
            // Reset limiter to the current flywheel RPM
            rpmAdjustLimiter.reset(shooter.flywheelRpmSupplier.getAsDouble());
        },
        () -> {
            double runAtRpm = 0.0; // Idle flywheel speed

            if (fmsAvailable.getAsBoolean() && hubActive.getAsBoolean() && inScoringZone.getAsBoolean())
            {
                runAtRpm = activeIdleRpm;
            }

            shooter.runAtRpm(rpmAdjustLimiter.calculate(runAtRpm));
        },
        shooter)
        .withName("autoIdleFlywheel");
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
