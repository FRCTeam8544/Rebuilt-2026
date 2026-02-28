package frc.robot.commands;

import frc.robot.subsystems.shooter.Shooter;

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

    private static final double DEADBAND = 0.1;
    private static final double FF_START_DELAY = 2.0; // Secs
    private static final double FF_RAMP_RATE = 0.1; // Volts/Sec

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
                shooter.stopFeed();
                shooter.stopShooter();
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

            shooter.runShooterOpenLoop();
            //shooter.runShooter(3000);
            
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
                                       Trigger feedAdjustDown,
                                       Trigger feedAdjustUp)
    {
        return Commands.run(
        () -> {

            final int feedNominalRpm = 300;
            final int rpmAdjustStep = 5;
            final int shooterNominalRpm = 3000;
            boolean adjustUp = rpmAdjustUp.getAsBoolean();
            boolean adjustDown = rpmAdjustDown.getAsBoolean();

           /// if (!resetShooterDefaults.getAsBoolean())
            {
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
            }
          //  else {
             //   shooter.resetShooterDefaultVoltage();
             //   shooter.resetFeedDefaultRpm();
        //    }

            if (shootTrigger.getAsBoolean())
            {
                shooter.runShooterOpenLoop();
            }
            else {
                shooter.stopShooter();
            }

            boolean feedAdjUp = feedAdjustUp.getAsBoolean();
            boolean feedAdjDown = feedAdjustDown.getAsBoolean();
            if (feedAdjDown ^ feedAdjUp) {
                if (feedAdjUp) {
                    shooter.tuneIncreaseFeedVoltage();
                    //shooter.feedRpmAdjust(rpmAdjustStep);
                }
                else {
                    shooter.tuneDecreaseFeedVoltage();
                   //shooter.feedRpmAdjust(-rpmAdjustStep);
                }
            }

            if (feedTrigger.getAsBoolean())
            {
               // shooter.runFeed(feedNominalRpm);
                shooter.runFeedOpenLoop();
            }
            else {
                shooter.stopFeed();
            }
        },
        shooter);
    } 

  /*  public static Command joystickVoltsShoot( Shooter shooter, 
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
    }*/

    
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
