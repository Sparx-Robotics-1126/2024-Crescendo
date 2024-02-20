package frc.team1126.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team1126.subsystems.sensors.Limelight;

public class SuperSystem extends SubsystemBase{

    private SwerveSubsystem swerve;
    private Climber climber;
    private Limelight limelight;
    private Arm arm;
     private CANdleSubsystem candle;
     private Shooter shooter;
     private Storage storage;


    public SuperSystem(SwerveSubsystem swerve, Climber climber,
                       Limelight limelight, Arm arm,
                       CANdleSubsystem candle, Shooter shooter,
                       Storage storage) {

        this.swerve = swerve;
        this.climber = climber;
        this.limelight = limelight;
        this.arm = arm;
        this.candle = candle;
        this.shooter = shooter;
        this.storage = storage;
    }

    @Override
    public void periodic() {

        if (storage.hasNote()) {
            if (limelight.inSpeakerRange(204) || limelight.inSpeakerRange(105)) {
                SmartDashboard.putBoolean("In Range", true);
                if (limelight.inSpeakerRange(204)) {
                    candle.setLEDState(CANdleSubsystem.LEDState.BLU);
                } else if (limelight.inSpeakerRange(105)) {
                    candle.setLEDState(CANdleSubsystem.LEDState.GREEN);
                } else {
                    candle.setLEDState(CANdleSubsystem.LEDState.YELLOW);
                }
            } else {
                SmartDashboard.putBoolean("In Range", false);
            }
        }
    }
}
