package frc.team1126.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team1126.Constants;
import frc.team1126.Constants.ArmConstants;

public class Arm extends SubsystemBase {

    private CANSparkMax armMotor;
    private CANSparkMax armMotorSlave;

    private RelativeEncoder armEncoder;
    private RelativeEncoder armEncoderSlave;

    private Pigeon2 armPigeon;

    private double power = 0;

    public Arm() {
        armMotor = new CANSparkMax(ArmConstants.MASTER_ID, CANSparkLowLevel.MotorType.kBrushless);
        armMotorSlave = new CANSparkMax(ArmConstants.SLAVE_ID, CANSparkLowLevel.MotorType.kBrushless);

        armMotorSlave.setInverted(true);
        armPigeon = new Pigeon2(ArmConstants.ARM_PIGEON_ID);

        configureMotor(armMotor, armMotorSlave);
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
        var pitch = armPigeon.getPitch().getValue();

        SmartDashboard.putNumber("arm Pitch", pitch);
        SmartDashboard.putNumber("Arm Power", power);
    }

    public void moveArm(double power) {
        this.power = power;
        armMotor.set(power);
    }

}