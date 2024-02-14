package frc.team1126.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team1126.Constants;
import frc.team1126.Constants.RotationConstants;

public class Rotation extends SubsystemBase {

    private CANSparkMax rotationMotor;
    private CANSparkMax rotationMotorSlave;

    private RelativeEncoder rotationEncoder;
    private RelativeEncoder rotationEncoderSlave;

    private Pigeon2 rotationPigeon;

    private double power=0;

    public Rotation() {
        rotationMotor = new CANSparkMax(RotationConstants.MASTER_ID, CANSparkLowLevel.MotorType.kBrushless);
        rotationMotorSlave = new CANSparkMax(RotationConstants.SLAVE_ID, CANSparkLowLevel.MotorType.kBrushless);

        rotationMotorSlave.setInverted(true);
        rotationPigeon = new Pigeon2(RotationConstants.ROTATION_PIGEON_ID);

        configureMotor(rotationMotor, rotationMotorSlave);
    }

    /**
     * Configures motors to follow one controller.
     *
     * @param master The controller to follow.
     * @param slaves The controllers that should follow master.
     */
    private static void configureMotor(CANSparkMax master, CANSparkMax... slaves) {
        master.restoreFactoryDefaults();
        master.set(0);
        master.setIdleMode(IdleMode.kCoast);
        master.enableVoltageCompensation(Constants.NOMINAL_VOLTAGE);
        master.setSmartCurrentLimit(Constants.MAX_CURRENT);

        for (CANSparkMax slave : slaves) {
            slave.restoreFactoryDefaults();
            slave.follow(master);
            slave.setIdleMode(IdleMode.kCoast);
            slave.setSmartCurrentLimit(Constants.MAX_CURRENT);
        }
    }

    @Override
    public void periodic() {
        var pitch = rotationPigeon.getPitch().getValue();

        SmartDashboard.putNumber("Rotation Pitch", pitch);
        SmartDashboard.putNumber("Arm Power", power);
    }

    public void moveArm(double power) {
        this.power = power;
        rotationMotor.set(power);
    }

}
