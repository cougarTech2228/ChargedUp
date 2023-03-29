package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.led.SegmentedLEDStrip;
import frc.robot.utils.led.SegmentedLEDStrip.ColorPattern;
import frc.robot.utils.led.SegmentedLEDStrip.GlowColor;
import frc.robot.utils.led.SegmentedLEDStrip.Speed;
import frc.robot.RobotContainer;
import frc.robot.utils.led.LEDBufferSegment;

public class LEDStripSubsystem extends SubsystemBase {

    private final static int LED_STRIP_PWM = 0;

    private static SegmentedLEDStrip m_ledStrip;

    private LEDBufferSegment left_upright_segment;
    private LEDBufferSegment top_segment;
    private LEDBufferSegment right_upright_segment;
    private LEDBufferSegment chassis_segment;
    private LEDBufferSegment whole_strip_segment;

    public void initTeleStrip() {
        m_ledStrip.clearSegments();
        left_upright_segment = new LEDBufferSegment("left_upright",0, 31 );
        //left_upright_segment.doRainbow();

        top_segment = new LEDBufferSegment("top",31, 12 );
        //top_segment.doGlow(GlowColor.Red);

        right_upright_segment = new LEDBufferSegment("right_upright",43, 25 );
        //right_upright_segment.doSnake(Speed.Fast, Color.kBlue, ColorPattern.SnakePacman);

        chassis_segment = new LEDBufferSegment("chassis",68, 82 );
        //chassis_segment.doMovingColors(Speed.Slow, ColorPattern.Patriotic);

        if (DriverStation.getAlliance() == Alliance.Blue) {
            chassis_segment.doGlow(SegmentedLEDStrip.GlowColor.Blue);
        } else if (DriverStation.getAlliance() == Alliance.Red) {
            chassis_segment.doGlow(SegmentedLEDStrip.GlowColor.Red);
        }
        
        m_ledStrip.addSegment(left_upright_segment);
        m_ledStrip.addSegment(top_segment);
        m_ledStrip.addSegment(right_upright_segment);
        m_ledStrip.addSegment(chassis_segment);

        m_ledStrip.printSegments();
    }

    public LEDStripSubsystem() {
        m_ledStrip  = new SegmentedLEDStrip(LED_STRIP_PWM);
        initTeleStrip();
    }

    @Override
    public void periodic() {
        m_ledStrip.renderString();
    }

    public void gripperLights(boolean isOpen){
        if(isOpen){
            if(!RobotContainer.isEndgame()){
                top_segment.doGlow(GlowColor.Green);
                right_upright_segment.doGlow(GlowColor.Green);
                left_upright_segment.doGlow(GlowColor.Green);
            } else {
                right_upright_segment.doGlow(GlowColor.Green);
                left_upright_segment.doGlow(GlowColor.Green);
            }
        } else {
            if(RobotContainer.isEndgame()){
                top_segment.doGlow(GlowColor.Red);
                right_upright_segment.doGlow(GlowColor.Red);
                left_upright_segment.doGlow(GlowColor.Red);
            } else {
                right_upright_segment.doGlow(GlowColor.Red);
                left_upright_segment.doGlow(GlowColor.Red);
            }
        }
    }

    public void autoPretty(){
        m_ledStrip.clearSegments();
        whole_strip_segment = new LEDBufferSegment("whole_strip", 0, 150);
        m_ledStrip.addSegment(whole_strip_segment);
        m_ledStrip.doRainbow(whole_strip_segment);
    }

    public void endGameLights(){
        top_segment.doMovingColors(Speed.Ludicrous, ColorPattern.Cougartech);
    }
}
