package frc.team1126.subsystems;

import java.util.Optional;

import com.ctre.phoenix.led.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team1126.Constants.CANdleConstants;

import static edu.wpi.first.wpilibj.DriverStation.Alliance.*;

public class CANdleSubsystem extends SubsystemBase {
    private final CANdle m_candle = new CANdle(CANdleConstants.CANDLE_ID);
    private final int LedCount = 300;
    public LEDState ledstate;
    private Animation m_toAnimate = null;
	XboxController m_controller;

	// private boolean m_clearAllAnims = false;
    // private boolean m_last5V = false;
    // private boolean m_animDirection = false;
    // private boolean m_setAnim = false;

    public enum AnimationTypes {
        ColorFlow,
        Fire,
        Larson,
        Rainbow,
        RgbFade,
        SingleFade,
        Strobe,
        Twinkle,
        TwinkleOff,
        SetAll,
        Empty,
        RedColorFlow,
        BlueColorFlow,
        YellowColorFlow,
        PurpleColorFlow,
        purpleStrobe
    }
    private AnimationTypes m_currentAnimation;
    public CANdleSubsystem() {
        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = true;
        configAll.disableWhenLOS = false;
        configAll.stripType = CANdle.LEDStripType.RGB;
        configAll.brightnessScalar = 0.1;
        m_candle.configAllSettings(configAll, 100);
        ledstate = LEDState.RED;

		m_candle.clearAnimation(0);

        if (RobotBase.isReal())
        {
            if (DriverStation.getAlliance().get() == Blue) {
                setLEDState(LEDState.BLU);
            } else {
                setLEDState(LEDState.RED);
            }
        }
    }

    public void setLEDState(LEDState state) {


        ledstate = state;
        // m_candle.setLEDs(state.r, state.g, state.b);
		// m_candle.animate(new LarsonAnimation(state.r, state.g, state.b, 0, .5, LedCount, LarsonAnimation.BounceMode.Front, 7));
		
		m_candle.clearAnimation(0);
		m_candle.setLEDs(state.r, state.g, state.b);
    }

//	public InstantCommand setLEDSTateCommand(LEDState state) {
//
//        // ledstate = state;
//        // m_candle.setLEDs(state.r, state.g, state.b);
//
//		return new InstantCommand(() -> setLEDSTate( state));
//
//    }

    public LEDState getLEDState() {

        return ledstate;

    }

    public enum LEDState {

        PURPLE(CANdleConstants.PURPLE_R, CANdleConstants.PURPLE_G, CANdleConstants.PURPLE_B),
        YELLOW(CANdleConstants.YELLOW_R, CANdleConstants.YELLOW_G, CANdleConstants.YELLOW_B),
        RED(CANdleConstants.RED_R, CANdleConstants.RED_G, CANdleConstants.RED_B),
		BLU(CANdleConstants.BLUE_R, CANdleConstants.BLUE_G, CANdleConstants.BLUE_B),
        GREEN(CANdleConstants.GREEN_R, CANdleConstants.GREEN_G, CANdleConstants.GREEN_B);

        public final int r;
        public final int g;
        public final int b;

        private LEDState(int r, int g, int b) {
            this.r = r;
            this.g = g;
            this.b = b;
        }

    }
    public void changeAnimation(AnimationTypes toChange) {
        m_currentAnimation = toChange;

        switch(toChange)
        {
            case ColorFlow:
                m_toAnimate = new ColorFlowAnimation(128, 20, 70, 0, 0.7, LedCount, ColorFlowAnimation.Direction.Forward);
                break;
            case Fire:
                m_toAnimate = new FireAnimation(0.5, 0.7, LedCount, 0.7, 0.5);
                break;
            case Larson:
                m_toAnimate = new LarsonAnimation(0, 255, 46, 0, .01, LedCount, LarsonAnimation.BounceMode.Front, 7);
                break;
            case Rainbow:
                m_toAnimate = new RainbowAnimation(1, 0.1, LedCount);
                break;
            case RgbFade:
                m_toAnimate = new RgbFadeAnimation(0.7, 0.4, LedCount);
                break;
            case SingleFade:
                m_toAnimate = new SingleFadeAnimation(50, 2, 200, 0, 0.5, LedCount);
                break;
            case Strobe:
                m_toAnimate = new StrobeAnimation(240, 10, 180, 0, 98.0 / 256.0, LedCount);
                break;
            case Twinkle:
                m_toAnimate = new TwinkleAnimation(30, 70, 60, 0, 0.4, LedCount, TwinkleAnimation.TwinklePercent.Percent6);
                break;
            case TwinkleOff:
                m_toAnimate = new TwinkleOffAnimation(70, 90, 175, 0, 0.8, LedCount, TwinkleOffAnimation.TwinkleOffPercent.Percent100);
                break;
            case SetAll:
                m_toAnimate = null;
                break;

            case RedColorFlow:
                m_toAnimate =  new ColorFlowAnimation(128, 0, 0, 0, 0.7, LedCount, ColorFlowAnimation.Direction.Forward);
                break;

            case BlueColorFlow:
                m_toAnimate =  new ColorFlowAnimation(0, 0, 128, 0, 0.7, LedCount, ColorFlowAnimation.Direction.Forward);
                break;
            case YellowColorFlow:
                m_toAnimate =  new ColorFlowAnimation(255,255,0, 0, 0.7, LedCount, ColorFlowAnimation.Direction.Forward);
                break;

            case PurpleColorFlow:
                m_toAnimate =  new ColorFlowAnimation(100,0,255, 0, 0.7, LedCount, ColorFlowAnimation.Direction.Forward);
                break;
            case Empty:
                break;
            case purpleStrobe:
            m_toAnimate = new StrobeAnimation(CANdleConstants.PURPLE_R,CANdleConstants.PURPLE_G, CANdleConstants.PURPLE_B, 0,  98.0 / 256.0, LedCount);
            default:
                break;
        }
        System.out.println("Changed to " + m_currentAnimation.toString());
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
} 