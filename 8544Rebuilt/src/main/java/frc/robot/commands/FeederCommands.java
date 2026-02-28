package frc.robot.commands;

import frc.robot.subsystems.Feeder.*;  //TODO move to new FeederCommands.java, clean up code
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class FeederCommands {

    private static final double DEADBAND = 0.1;
    private static final double FF_START_DELAY = 2.0; // Secs
    private static final double FF_RAMP_RATE = 0.1; // Volts/Sec

    private FeederCommands() {}
/* 
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
            */

    public static Command stopMotors(Feeder feeder) {
        return Commands.run(
        () -> {
                feeder.stopFeed();
            },
            feeder);
    }

    // Use this command to tune Ks by increasing voltage until the flywheel
    // begins to slightly turn, then back off a bit.
    // This term will be used in the PID with feedforward control later.
    public static Command openVoltageControl(Feeder feeder,
                                         Trigger feedTrigger,
                                         Trigger increaseFeedVoltageTrigger,
                                         Trigger decreaseFeedVoltageTrigger)
    {
        return Commands.run(
        () -> {


            boolean increaseFeedVolt = increaseFeedVoltageTrigger.getAsBoolean();
            boolean decreaseFeedVolt = decreaseFeedVoltageTrigger.getAsBoolean();
            if (increaseFeedVolt ^ decreaseFeedVolt) {
                if (increaseFeedVoltageTrigger.getAsBoolean())
                {
                    feeder.tuneIncreaseFeedVoltage();
                }
                else
                {
                    feeder.tuneDecreaseFeedVoltage();
                }
            }

            feeder.runFeedOpenLoopReverse();
            //shooter.runShooter(3000);
            
            if (feedTrigger.getAsBoolean())
            {
                feeder.runFeedOpenLoop();
            }
            else {
                feeder.runFeedOpenLoop(0.0);
            }


            feeder.runFeedOpenLoop();
            //shooter.runShooter(3000);
            
            if (feedTrigger.getAsBoolean())
            {
                feeder.runFeedOpenLoop();
            }
            else {
                feeder.runFeedOpenLoop(0.0);
            }
        },
        feeder);



    }

    public static Command buttonFeed( Feeder feeder,
                                       Trigger feedTrigger,
                                       Trigger reversefeedTrigger,
                                       Trigger feedAdjustDown,
                                       Trigger feedAdjustUp
                                       )
    {
        return Commands.run(
        () -> {

            final int feedNominalRpm = 300;
            final int rpmAdjustStep = 5;



            boolean feedAdjUp = feedAdjustUp.getAsBoolean();
            boolean feedAdjDown = feedAdjustDown.getAsBoolean();
            if (feedAdjDown ^ feedAdjUp) {
                if (feedAdjUp) {
                    feeder.tuneIncreaseFeedVoltage();
                    //shooter.feedRpmAdjust(rpmAdjustStep);
                }
                else {
                    feeder.tuneDecreaseFeedVoltage();
                   //shooter.feedRpmAdjust(-rpmAdjustStep);
                }
            }

            if (feedTrigger.getAsBoolean())
            {
               // shooter.runFeed(feedNominalRpm);
                feeder.runFeedOpenLoop();
            }
            else {
                feeder.stopFeed();
            }
        },
        feeder);
    } 


}


