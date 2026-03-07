package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Feeder.*;
import frc.robot.subsystems.leds.Leds;

import java.util.Vector;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class FeederCommands {

    //private static final double FF_START_DELAY = 2.0; // Secs
    //private static final double FF_RAMP_RATE = 0.1; // Volts/Sec

    private FeederCommands() {}

    public static Command stopMotors(Feeder feeder) {
        return Commands.run(
        () -> {
                feeder.stopMotors();
            },
            feeder);
    }

    // Use this command to tune Ks by increasing voltage until the flywheel
    // begins to slightly turn, then back off a bit.
    // This term will be used in the PID with feedforward control later.
    public static Command openVoltageControl(
        Feeder feeder,
        Trigger feedTrigger,
        Trigger reverseFeedTrigger,
        Trigger decreaseFeedVoltageTrigger,
        Trigger increaseFeedVoltageTrigger)
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

            if (feedTrigger.getAsBoolean())
            {
                if (reverseFeedTrigger.getAsBoolean()) {
                    feeder.runOpenLoopReverse();
                }
                else {
                    feeder.runOpenLoop();
                }
            }
            else {
                feeder.runOpenLoop(0.0);
            }
        },
        feeder);
    }

    public static Command buttonFeed( Feeder feeder,
                                       Trigger feedTrigger,
                                       Trigger reversefeedTrigger,
                                       Trigger feedAdjustDown,
                                       Trigger feedAdjustUp,
                                       Leds leds
                                       )
    {
        final double nominalRpm = 300; // Wheel RPM request
        final double reverseRpmFactor = -0.5; // Reverse half speed
        final double rpmStep = 20.0 / Constants.tickUpdatesPerSecond;

        // Slightly awkward.... lamda storage of currentRpmAdjust across ticks
        Vector<Double> currentRpmAdjust = new Vector<>(1);
        currentRpmAdjust.add(0.0);

        return Commands.run(
        () -> {

            boolean feedAdjUp = feedAdjustUp.getAsBoolean();
            boolean feedAdjDown = feedAdjustDown.getAsBoolean();
            if (feedAdjDown ^ feedAdjUp) {
                if (feedAdjUp) {
                    double newFeedRpm = currentRpmAdjust.firstElement().doubleValue() + rpmStep;
                    currentRpmAdjust.clear();
                    currentRpmAdjust.add(newFeedRpm);
                }
                else {
                    double newFeedRpm = currentRpmAdjust.firstElement().doubleValue() - rpmStep;
                    currentRpmAdjust.clear();
                    currentRpmAdjust.add(newFeedRpm);
                }
            }

            if (feedTrigger.getAsBoolean())
            {
                double rpm = nominalRpm + currentRpmAdjust.firstElement().doubleValue();
                if (reversefeedTrigger.getAsBoolean()) {
                    leds.setMechanicalState(Leds.MechanicalState.NONE);
                    feeder.runAtRpm(rpm * reverseRpmFactor);
                }
                else {
                    leds.setMechanicalState(Leds.MechanicalState.SHOOTING);
                    feeder.runAtRpm(rpm);
                }
            }
            else {
                leds.setMechanicalState(Leds.MechanicalState.NONE);
                feeder.stopMotors();
            }
        },
        feeder);
    }

}


