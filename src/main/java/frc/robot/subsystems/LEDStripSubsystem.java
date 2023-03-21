package frc.robot.subsystems;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.led.SegmentedLEDStrip;
import frc.robot.utils.led.SegmentedLEDStrip.ColorPattern;
import frc.robot.utils.led.SegmentedLEDStrip.GlowColor;
import frc.robot.utils.led.SegmentedLEDStrip.Speed;
import frc.robot.utils.led.LEDBufferSegment;

public class LEDStripSubsystem extends SubsystemBase {

    private final static int LED_STRIP_PWM = 0;

    private static SegmentedLEDStrip m_ledStrip = new SegmentedLEDStrip(LED_STRIP_PWM);

    private LEDBufferSegment left_upright_segment;
    private LEDBufferSegment top_segment;
    private LEDBufferSegment right_upright_segment;
    private LEDBufferSegment chassis_segment;

    private void initStrip() {
        left_upright_segment = new LEDBufferSegment("left_upright",0, 31 );
        left_upright_segment.doRainbow();

        top_segment = new LEDBufferSegment("top",31, 12 );
        top_segment.doGlow(GlowColor.Red);

        right_upright_segment = new LEDBufferSegment("right_upright",43, 25 );
        right_upright_segment.doSnake(Speed.Fast, Color.kBlue, ColorPattern.SnakePacman);

        chassis_segment = new LEDBufferSegment("chassis",68, 82 );
        chassis_segment.doMovingColors(Speed.Slow, ColorPattern.Patriotic);

        m_ledStrip.addSegment(left_upright_segment);
        m_ledStrip.addSegment(top_segment);
        m_ledStrip.addSegment(right_upright_segment);
        m_ledStrip.addSegment(chassis_segment);

    }

    public LEDStripSubsystem() {
        initStrip();
    }

    // public void resetStrip() {
    //     m_ledStrip.setColor(Color.kBlack);
    //     m_ledStripState = StripState.DoingNothing;
    // }

    // public void glow(GlowColor glowColor) {
    //     m_currentGlowColor = glowColor;
    //     m_ledStripState = StripState.Glowing;
    // }

    // public void snakeColors(Speed speed, Color color, Color... colorArray) {
    //     m_currentSpeed = speed;
    //     m_currentBackgroundColor = color;
    //     m_currentColorArray = colorArray;
    //     m_ledStripState = StripState.SnakingColors;
    // }

    // public void snakePattern(Speed speed, Color color, ColorPattern colorPattern) {
    //     m_currentSpeed = speed;
    //     m_currentBackgroundColor = color;
    //     m_currentColorPattern = colorPattern;
    //     m_ledStripState = StripState.SnakingPattern;
    // }

    // public void moveColor(Speed speed, Color... colorArray) {
    //     m_currentSpeed = speed;
    //     m_currentColorArray = colorArray;
    //     m_ledStripState = StripState.MovingColor;
    // }

    // public void moveColorPattern(Speed speed, ColorPattern colorPattern) {
    //     m_currentSpeed = speed;
    //     m_currentColorPattern = colorPattern;
    //     m_ledStripState = StripState.MovingColorPattern;
    // }

    // public void rainbow() {
    //     m_ledStripState = StripState.Rainbow;
    // }

    @Override
    public void periodic() {
        m_ledStrip.renderString();
        // switch (m_ledStripState) {
        //     case DoingNothing: {
        //         initStrip();
        //     }
        //         break;
        //     case MovingColor: {
        //         m_ledStrip.doMovingColors(m_currentSpeed, m_currentColorArray);
        //     }
        //         break;
        //     case MovingColorPattern: {
        //         m_ledStrip.doMovingColors(m_currentSpeed, m_currentColorPattern);
        //     }
        //         break;
        //     case SnakingColors: {
        //         m_ledStrip.doSnake(m_currentSpeed, m_currentBackgroundColor, m_currentColorArray);
        //     }
        //         break;
        //     case SnakingPattern: {
        //         m_ledStrip.doSnake(m_currentSpeed, m_currentBackgroundColor, m_currentColorPattern);
        //     }
        //         break;
        //     case Rainbow: {
        //         m_ledStrip.doRainbow();
        //     }
        //         break;
        //     case Glowing: {
        //         m_ledStrip.doGlow(m_currentGlowColor);
        //     }
        //         break;
        //     default:
        //         initStrip();
        // }

    }
}
