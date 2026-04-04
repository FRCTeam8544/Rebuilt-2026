package frc.robot.subsystems.leds;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;

import frc.robot.generated.TunerConstants;

public class LedIOCANdle implements LedIO {

    private static final int CAN_ID = 20;

    // External LED strip only: skip the 8 onboard CANdle LEDs (indices 0–7)
    private static final int LED_START = 8;
    // 160 external LEDs starting at index 8
    private static final int LED_END = 167;

    private final CANdle candle;

    private final SolidColor solidRequest = new SolidColor(LED_START, LED_END);
    private final SingleFadeAnimation breathRequest = new SingleFadeAnimation(LED_START, LED_END);
    private final StrobeAnimation strobeRequest = new StrobeAnimation(LED_START, LED_END);
    private final ColorFlowAnimation waveRequest =
            new ColorFlowAnimation(LED_START, LED_END)
                    .withDirection(AnimationDirectionValue.Forward);

    private String currentAnimation = "NONE";
    private int currentR = 0;
    private int currentG = 0;
    private int currentB = 0;

    public LedIOCANdle() {
        candle = new CANdle(CAN_ID, TunerConstants.kCANBus);

        CANdleConfiguration config = new CANdleConfiguration();
        config.LED.StripType = StripTypeValue.GRB;
        candle.getConfigurator().apply(config);
    }

    @Override
    public void updateInputs(LedIOInputs inputs) {
        inputs.connected = candle.isConnected();
        inputs.activeAnimation = currentAnimation;
        inputs.colorR = currentR;
        inputs.colorG = currentG;
        inputs.colorB = currentB;
    }

    @Override
    public void setSolid(int r, int g, int b) {
        currentAnimation = "SOLID";
        updateColor(r, g, b);
        candle.setControl(solidRequest.withColor(new RGBWColor(r, g, b)));
    }

    @Override
    public void setBreath(int r, int g, int b, double speedHz) {
        currentAnimation = "BREATH";
        updateColor(r, g, b);
        candle.setControl(
                breathRequest
                        .withColor(new RGBWColor(r, g, b))
                        .withFrameRate(speedHz));
    }

    @Override
    public void setStrobe(int r, int g, int b, double speedHz) {
        currentAnimation = "STROBE";
        updateColor(r, g, b);
        candle.setControl(
                strobeRequest
                        .withColor(new RGBWColor(r, g, b))
                        .withFrameRate(speedHz));
    }

    @Override
    public void setWave(int r, int g, int b, double speedHz) {
        currentAnimation = "WAVE";
        updateColor(r, g, b);
        candle.setControl(
                waveRequest
                        .withColor(new RGBWColor(r, g, b))
                        .withFrameRate(speedHz));
    }

    private void updateColor(int r, int g, int b) {
        currentR = r;
        currentG = g;
        currentB = b;
    }
}
