package frc.team1126.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.team1126.Constants.ClimberConstants.*;

public class Climber extends SubsystemBase {

    private CANSparkMax m_motorLeft;
    private CANSparkMax m_motorRight;

    private RelativeEncoder m_leftEncoder;
    private RelativeEncoder m_rightEncoder;

    // private SwerveIMU imu;

    private DigitalInput m_leftHome;
    private DigitalInput m_rightHome;

    private double m_leftPower = 0.0;
    private double m_rightPower = 0.0;

    public Climber() {

        m_leftHome = new DigitalInput(LEFT_DIGITAL_INPUT);
        m_rightHome = new DigitalInput(RIGHT_DIGITAL_INPUT);

        // imu= swerveSubsystem.getSwerveIMU();
        m_motorLeft = new CANSparkMax(MOTOR_LEFT_ID, CANSparkLowLevel.MotorType.kBrushless);
        m_motorRight = new CANSparkMax(MOTOR_RIGHT_ID, CANSparkLowLevel.MotorType.kBrushless);

        m_motorLeft.setInverted(true);
        m_motorRight.setInverted(true);
        m_leftEncoder = m_motorLeft.getEncoder();
        m_rightEncoder = m_motorRight.getEncoder();

        m_rightEncoder.setPositionConversionFactor(kEncoderDistanceConversionFactor);
        m_leftEncoder.setPositionConversionFactor(kEncoderDistanceConversionFactor);
        m_leftEncoder
                .setVelocityConversionFactor(Math.PI * K_WHEEL_DIAMETER_METERS / K_GEAR_RATIO / 60.0);
        m_rightEncoder
                .setVelocityConversionFactor(Math.PI * K_WHEEL_DIAMETER_METERS / K_GEAR_RATIO / 60.0);

        m_leftEncoder.setPosition(0);
        m_rightEncoder.setPosition(0);

    }

    @Override
    public void periodic() {
        double leftHeight = m_leftEncoder.getPosition();
        double rightHeight = m_rightEncoder.getPosition();
        double averageHeight = (leftHeight + rightHeight) / 2.0;

        SmartDashboard.putNumber("Climber Height", averageHeight);

        SmartDashboard.putNumber("Left height", m_leftEncoder.getPosition());
        SmartDashboard.putNumber("Right height", m_rightEncoder.getPosition());
        SmartDashboard.putBoolean("Left Sensor", !m_leftHome.get());
        SmartDashboard.putBoolean("Right Sensor", !m_rightHome.get());
        SmartDashboard.putNumber("Left Power", m_leftPower);
        SmartDashboard.putNumber("Right Power", m_rightPower);
    }

    public Command moveLeftClimber(double leftY) {
        return this.run(() -> setLeftPower(leftY));
    }

    public Command moveRightClimber(double rightY) {
        return this.run(() -> setLeftPower(rightY));
    }

    public void setLeftPower(double leftY) {
        m_leftPower = leftY;
        m_motorLeft.set(leftY);
    }

    public void setRightPower(double rightY) {
        m_rightPower = rightY;
        m_motorRight.set(rightY);
    }

    public double getLeftPosition() {
        return m_leftEncoder.getPosition();
    }

    public double getRightPosition() {
        return m_rightEncoder.getPosition();
    }

    public boolean isLeftHome() {
        // if (!m_leftHome.get()){
        //     m_leftEncoder.setPosition(0.0);
        //     m_motorLeft.set(0);
        // }
        return m_leftHome.get();
    }

    public boolean isRightHome() {
        // if(!m_rightHome.get()) {
        //     m_rightEncoder.setPosition(0.0);
        //     m_motorRight.set(0);
        // }
        return m_rightHome.get();
    }

    public void zeroLeft() {
        m_leftEncoder.setPosition(0);
    }

    public void zeroRight() {
        m_rightEncoder.setPosition(0);
    }


}
