package frc.robot.subsystems.leds;

import org.littletonrobotics.junction.AutoLog;

public interface LedIO {

    @AutoLog
    public static class LedIOInputs {
        public boolean connected = false;
        public String activeAnimation = "NONE";
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(LedIOInputs inputs) {}

    /** Sets all LEDs to a solid color. */
    public default void setSolid(int r, int g, int b) {}

    /**
     * Fades all LEDs in and out of the given color (breath effect).
     *
     * @param r red component [0, 255]
     * @param g green component [0, 255]
     * @param b blue component [0, 255]
     * @param speedHz animation frame rate in Hz [2, 1000]
     */
    public default void setBreath(int r, int g, int b, double speedHz) {}

    /**
     * Strobes all LEDs between the given color and off.
     *
     * @param r red component [0, 255]
     * @param g green component [0, 255]
     * @param b blue component [0, 255]
     * @param speedHz animation frame rate in Hz [1, 500]
     */
    public default void setStrobe(int r, int g, int b, double speedHz) {}

    /**
     * Flows a color along the LED strip, creating a wave effect.
     *
     * @param r red component [0, 255]
     * @param g green component [0, 255]
     * @param b blue component [0, 255]
     * @param speedHz animation frame rate in Hz [2, 1000]
     */
    public default void setWave(int r, int g, int b, double speedHz) {}
}
