package frc.team1126.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team1126.subsystems.sensors.Limelight;

public class SuperSystem extends SubsystemBase{

    private SwerveSubsystem m_swerve;
    private Climber m_climber;
    private Limelight m_limelight;
    private Arm m_arm;
    private CANdleSubsystem m_candle;
    private Shooter m_shooter;
    private Storage m_storage;


    public SuperSystem(SwerveSubsystem swerve, Climber climber,
                       Limelight limelight, Arm arm,
                       CANdleSubsystem candle, Shooter shooter,
                       Storage storage) {

        m_swerve = swerve;
        m_climber = climber;
        m_limelight = limelight;
        m_arm = arm;
        m_candle = candle;
        m_shooter = shooter;
        m_storage = storage;
    }

    @Override
    public void periodic() {

        if (m_storage.hasNote()) {
            if (m_limelight.inSpeakerRange(204) || m_limelight.inSpeakerRange(105)) {
                SmartDashboard.putBoolean("In Range", true);
                if (m_limelight.inSpeakerRange(204)) {
                    m_candle.setLEDState(CANdleSubsystem.LEDState.BLU);
                } else if (m_limelight.inSpeakerRange(105)) {
                    m_candle.setLEDState(CANdleSubsystem.LEDState.GREEN);
                } else {
                    m_candle.setLEDState(CANdleSubsystem.LEDState.YELLOW);
                }
            } else {
                SmartDashboard.putBoolean("In Range", false);
            }
        }
    }
}
