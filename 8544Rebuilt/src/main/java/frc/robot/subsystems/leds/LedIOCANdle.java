package frc.robot.subsystems.leds;

import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.RGBWColor;

import frc.robot.generated.TunerConstants;

public class LedIOCANdle implements LedIO {

    private static final int CAN_ID = 20;

    // External LED strip only: skip the 8 onboard CANdle LEDs (indices 0–7)
    private static final int LED_START = 8;
    // 100 external LEDs starting at index 8
    private static final int LED_END = 107;

    private final CANdle candle;

    private final SolidColor solidRequest = new SolidColor(LED_START, LED_END);
    private final SingleFadeAnimation breathRequest = new SingleFadeAnimation(LED_START, LED_END);
    private final StrobeAnimation strobeRequest = new StrobeAnimation(LED_START, LED_END);
    private final ColorFlowAnimation waveRequest =
            new ColorFlowAnimation(LED_START, LED_END)
                    .withDirection(AnimationDirectionValue.Forward);

    private String currentAnimation = "NONE";

    public LedIOCANdle() {
        candle = new CANdle(CAN_ID, TunerConstants.kCANBus);
    }

    @Override
    public void updateInputs(LedIOInputs inputs) {
        inputs.connected = candle.isConnected();
        inputs.activeAnimation = currentAnimation;
    }

    @Override
    public void setSolid(int r, int g, int b) {
        currentAnimation = "SOLID";
        candle.setControl(solidRequest.withColor(new RGBWColor(r, g, b)));
    }

    @Override
    public void setBreath(int r, int g, int b, double speedHz) {
        currentAnimation = "BREATH";
        candle.setControl(
                breathRequest
                        .withColor(new RGBWColor(r, g, b))
                        .withFrameRate(speedHz));
    }

    @Override
    public void setStrobe(int r, int g, int b, double speedHz) {
        currentAnimation = "STROBE";
        candle.setControl(
                strobeRequest
                        .withColor(new RGBWColor(r, g, b))
                        .withFrameRate(speedHz));
    }

    @Override
    public void setWave(int r, int g, int b, double speedHz) {
        currentAnimation = "WAVE";
        candle.setControl(
                waveRequest
                        .withColor(new RGBWColor(r, g, b))
                        .withFrameRate(speedHz));
    }
}
