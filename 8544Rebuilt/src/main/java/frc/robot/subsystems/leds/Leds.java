package frc.robot.subsystems.leds;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Leds extends SubsystemBase {

    // --- Timing ---
    private static final double STARTUP_DURATION_SECONDS = 30.0;
    private static final double ENDGAME_TIME_SECONDS = 30.0;

    // --- Thresholds ---
    private static final double LOW_BATTERY_VOLTAGE_VOLTS = 11.0;

    // --- Animation speeds (Hz = frames per second) ---
    // Breath: 1 frame = 1% brightness change, full cycle = 200 frames
    private static final double BREATH_SPEED_HZ = 150.0; // ~1.3s per cycle
    private static final double STROBE_SLOW_HZ = 5.0; // 1 frame = on/off toggle
    private static final double STROBE_RAPID_HZ = 10.0; // 1 frame = on/off toggle
    // Wave: 1 frame = 1 LED advance, full traversal = 160 frames
    private static final double WAVE_NORMAL_HZ = 200.0; // ~0.8s per traversal
    private static final double WAVE_FAST_HZ = 400.0; // ~0.4s per traversal

    // --- Colors [R, G, B] ---
    private static final int[] WHITE       = {150, 150, 150}; //was full brightness
    private static final int[] RED         = {170,   0,   0};
    private static final int[] BLUE        = {  0,   0, 255};
    private static final int[] BLUE_PURPLE = {75,   0, 150}; // blue-purple blend
    private static final int[] ORANGE_RED  = {150,  40,   0}; // CSS OrangeRed
    private static final int[] GREEN       = {  0, 150,   0};
    private static final int[] YELLOW      = {200, 200,   0};

    /**
     * Mechanical action states signalled by commands.
     * The subsystem merges these with robot-mode state to determine the final animation.
     */
    public enum MechanicalState {
        NONE,
        INTAKING,
        IN_LAUNCH_TOLERANCE,
        SHOOTING,
        IN_CLIMB_TOLERANCE,
        CLIMBING
    }

    private enum LedState {
        STARTUP,
        E_STOPPED,
        DISABLED_LOW_BATTERY,
        DISABLED_NO_ALLIANCE,
        DISABLED_BLUE,
        DISABLED_RED,
        AUTO,
        TELEOP_DEFAULT,
        TELEOP_ENDGAME,
        INTAKING,
        IN_LAUNCH_TOLERANCE,
        SHOOTING,
        IN_CLIMB_TOLERANCE,
        CLIMBING
    }

    private final LedIO ledIO;
    private final LedIOInputsAutoLogged ledInputs = new LedIOInputsAutoLogged();

    private final double startupTimestamp;
    private MechanicalState requestedMechanicalState = MechanicalState.NONE;

    /**
     * Creates the LED subsystem.
     *
     * @param io the LED IO implementation
     */
    public Leds(LedIO io) {
        this.ledIO = io;
        this.startupTimestamp = Timer.getFPGATimestamp();
    }

    /**
     * Called by commands to signal the current mechanical action.
     * Set to {@link MechanicalState#NONE} when no action is active.
     *
     * @param state the current mechanical state
     */
    public void setMechanicalState(MechanicalState state) {
        requestedMechanicalState = state;
    }

    @Override
    public void periodic() {
        ledIO.updateInputs(ledInputs);
        Logger.processInputs("Leds", ledInputs);

        LedState state = computeState();
        Logger.recordOutput("Leds/State", state.name());
        Logger.recordOutput("Leds/MechanicalState", requestedMechanicalState.name());
        Logger.recordOutput("Leds/AllianceDetected", DriverStation.getAlliance().isPresent());
        Logger.recordOutput(
                "Leds/Alliance", DriverStation.getAlliance().map(Enum::name).orElse("NONE"));
        Logger.recordOutput(
                "Leds/InStartup",
                Timer.getFPGATimestamp() - startupTimestamp < STARTUP_DURATION_SECONDS);

        applyAnimation(state);
    }

    private LedState computeState() {
        // 1. E-stopped — always highest priority for safety
        if (DriverStation.isEStopped()) {
            return LedState.E_STOPPED;
        }

        // 2. Startup — first 30 seconds of robot code running
        if (Timer.getFPGATimestamp() - startupTimestamp < STARTUP_DURATION_SECONDS) {
            return LedState.STARTUP;
        }

        // 3. Disabled sub-states
        if (DriverStation.isDisabled()) {
            if (RobotController.getBatteryVoltage() < LOW_BATTERY_VOLTAGE_VOLTS) {
                return LedState.DISABLED_LOW_BATTERY;
            }
            var alliance = DriverStation.getAlliance();
            if (alliance.isEmpty()) {
                return LedState.DISABLED_NO_ALLIANCE;
            }
            return alliance.get() == DriverStation.Alliance.Blue
                    ? LedState.DISABLED_BLUE
                    : LedState.DISABLED_RED;
        }

        // 4. Autonomous
        if (DriverStation.isAutonomousEnabled()) {
            return LedState.AUTO;
        }

        // 5. Teleop — mechanical state (set by commands) takes priority, then timing checks
        if (DriverStation.isTeleopEnabled()) {
            return switch (requestedMechanicalState) {
                case CLIMBING            -> LedState.CLIMBING;
                case IN_CLIMB_TOLERANCE  -> LedState.IN_CLIMB_TOLERANCE;
                case SHOOTING            -> LedState.SHOOTING;
                case IN_LAUNCH_TOLERANCE -> LedState.IN_LAUNCH_TOLERANCE;
                case INTAKING            -> LedState.INTAKING;
                case NONE -> {
                    double matchTime = DriverStation.getMatchTime();
                    yield (matchTime >= 0 && matchTime <= ENDGAME_TIME_SECONDS)
                            ? LedState.TELEOP_ENDGAME
                            : LedState.TELEOP_DEFAULT;
                }
            };
        }

        return LedState.TELEOP_DEFAULT;
    }

    private void applyAnimation(LedState state) {
        switch (state) {
            case STARTUP ->
                ledIO.setBreath(WHITE[0], WHITE[1], WHITE[2], BREATH_SPEED_HZ);

            case E_STOPPED ->
                ledIO.setSolid(RED[0], RED[1], RED[2]);

            case DISABLED_LOW_BATTERY ->
                ledIO.setStrobe(ORANGE_RED[0], ORANGE_RED[1], ORANGE_RED[2], STROBE_SLOW_HZ);

            case DISABLED_NO_ALLIANCE ->
                ledIO.setWave(BLUE_PURPLE[0], BLUE_PURPLE[1], BLUE_PURPLE[2], WAVE_NORMAL_HZ);

            case DISABLED_BLUE ->
                ledIO.setWave(BLUE[0], BLUE[1], BLUE[2], WAVE_NORMAL_HZ);

            case DISABLED_RED ->
                ledIO.setWave(RED[0], RED[1], RED[2], WAVE_NORMAL_HZ);

            case AUTO -> {
                int[] c = getAllianceColor();
                ledIO.setStrobe(c[0], c[1], c[2], STROBE_SLOW_HZ);
            }

            case TELEOP_DEFAULT ->
                ledIO.setWave(BLUE_PURPLE[0], BLUE_PURPLE[1], BLUE_PURPLE[2], WAVE_NORMAL_HZ);

            case TELEOP_ENDGAME -> {
                int[] c = getAllianceColor();
                ledIO.setWave(c[0], c[1], c[2], WAVE_FAST_HZ);
            }

            case INTAKING ->
                ledIO.setStrobe(BLUE_PURPLE[0], BLUE_PURPLE[1], BLUE_PURPLE[2], STROBE_SLOW_HZ);

            case IN_LAUNCH_TOLERANCE ->
                ledIO.setWave(GREEN[0], GREEN[1], GREEN[2], WAVE_FAST_HZ);

            case SHOOTING ->
                ledIO.setStrobe(GREEN[0], GREEN[1], GREEN[2], STROBE_RAPID_HZ);

            case IN_CLIMB_TOLERANCE ->
                ledIO.setWave(YELLOW[0], YELLOW[1], YELLOW[2], WAVE_FAST_HZ);

            case CLIMBING ->
                ledIO.setStrobe(YELLOW[0], YELLOW[1], YELLOW[2], STROBE_RAPID_HZ);
        }
    }

    /** Returns RED or BLUE based on the current alliance, defaulting to BLUE. */
    private static int[] getAllianceColor() {
        var alliance = DriverStation.getAlliance();
        boolean isRed = alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
        return isRed ? RED : BLUE;
    }
}
