package frc.robot.utils.led;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

import frc.robot.utils.led.SegmentedLEDStrip.ColorPattern;
import frc.robot.utils.led.SegmentedLEDStrip.GlowColor;
import frc.robot.utils.led.SegmentedLEDStrip.PercentDirection;
import frc.robot.utils.led.SegmentedLEDStrip.Speed;
import frc.robot.utils.led.SegmentedLEDStrip.StripEffect;


public class LEDBufferSegment extends AddressableLEDBuffer{
    public final String name;
    public final int start;
    public final int size;

    public Speed speed;
    public int m_rainbowFirstPixelHue = 0;

    public int m_snakeLoopIndex = 0;
    public int m_snakeCounter = 0;

    public int m_movingColorsCounter = 0;
    public int m_movingColorIndex = 0;
    public ColorPattern colorPattern;
    public Color backgroundColor;

    public int m_glowCounter = 0;
    public double m_glowIndex = 0;
    public GlowColor glowColor;
    public boolean m_isGlowReverse = false;

    public int m_percent;
    public Color m_color;
    public PercentDirection m_percentDirection;

    public StripEffect m_stripEffect = StripEffect.DoingNothing;
    /**
     * Create a new buffer segment
     * @param name A label for this segment (used in debugging)
     * @param start The pixel number on the strip where this buffer start
     * @param size The number of pixels in the segment
     */
    public LEDBufferSegment(String name, int start, int size) {
        super(size);
        this.name = name;
        this.start = start;
        this.size = size;
    }

    public void doMovingColors(Speed speed, ColorPattern color) {
        this.speed = speed;
        this.colorPattern = color;
        m_stripEffect = StripEffect.MovingColor;
    }

    public void doSnake(Speed speed, Color backgroundColor, ColorPattern snakeColorPattern) {
        this.speed = speed;
        this.backgroundColor = backgroundColor;
        this.colorPattern = snakeColorPattern;
        m_stripEffect = StripEffect.SnakingColors;
    }

    public void doGlow(GlowColor glowColor) {
        this.glowColor = glowColor;
        m_stripEffect = StripEffect.Glowing;
    }

    public void doRainbow() {
        m_stripEffect = StripEffect.Rainbow;
    }

    /**
     * Resets all the moving color variables, allows the changing of speed for
     * example.
     */
    public void reset() {
        m_rainbowFirstPixelHue = 0;

        m_snakeLoopIndex = 0;
        m_snakeCounter = 0;

        m_movingColorsCounter = 0;
        m_movingColorIndex = 0;

        m_glowCounter = 0;
    }

    public void doPercent(int percent, Color color, PercentDirection direction){
        m_percent = percent;
        m_color = color;
        m_percentDirection = direction;
        m_stripEffect = StripEffect.Percent;
    }
}
