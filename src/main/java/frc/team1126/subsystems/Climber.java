package frc.team1126.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.lib.swervelib.imu.SwerveIMU;
import static frc.team1126.Constants.ClimberConstants.*;
import frc.team1126.Constants.ShooterConstants;

public class Climber extends SubsystemBase {

    private CANSparkMax motorLeft;
    private CANSparkMax motorRight;

    private RelativeEncoder m_leftEncoder;
    private RelativeEncoder m_rightEncoder;

    // private SwerveIMU imu;

    private DigitalInput leftHome;
    private DigitalInput rightHome;

    private double leftPower = 0.0;
    private double rightPower = 0.0;

    public Climber() {

        leftHome = new DigitalInput(LEFT_DIGITAL_INPUT);
        rightHome = new DigitalInput(RIGHT_DIGITAL_INPUT);

        // imu= swerveSubsystem.getSwerveIMU();
        motorLeft = new CANSparkMax(MOTOR_LEFT_ID, CANSparkLowLevel.MotorType.kBrushless);
        motorRight = new CANSparkMax(MOTOR_RIGHT_ID, CANSparkLowLevel.MotorType.kBrushless);

        motorLeft.setInverted(true);
        motorRight.setInverted(true);
        m_leftEncoder = motorLeft.getEncoder();
        m_rightEncoder = motorRight.getEncoder();

        m_rightEncoder.setPositionConversionFactor(kEncoderDistanceConversionFactor);
        m_leftEncoder.setPositionConversionFactor(kEncoderDistanceConversionFactor);
        m_leftEncoder
                .setVelocityConversionFactor(Math.PI * kWheelDiameterMeters / kGearRatio / 60.0);
        m_rightEncoder
                .setVelocityConversionFactor(Math.PI * kWheelDiameterMeters / kGearRatio / 60.0);

        m_leftEncoder.setPosition(0);
        m_rightEncoder.setPosition(0);

    }

    @Override
    public void periodic() {
        double leftHeight = m_leftEncoder.getPosition();
        double rightHeight = m_rightEncoder.getPosition();
        double averageHeight = (leftHeight + rightHeight) / 2.0;

        SmartDashboard.putNumber("Climber Height", averageHeight);
        // SmartDashboard.putNumber("Roll", imu.getRoll());

        SmartDashboard.putNumber("Left height", m_leftEncoder.getPosition());
        SmartDashboard.putNumber("Right height", m_rightEncoder.getPosition());
        SmartDashboard.putBoolean("Left Sensor", !leftHome.get());
        SmartDashboard.putBoolean("Right Sensor", !rightHome.get());
        SmartDashboard.putNumber("Left Power", leftPower);
        SmartDashboard.putNumber("Right Power", rightPower);
    }

    public Command moveLeftClimber(double leftY) {
        return this.run(() -> setLeftPower(leftY));
    }

    public Command moveRightClimber(double rightY) {
        return this.run(() -> setLeftPower(rightY));
    }

    // public Command moveClimberWithIMU(double leftY, double rightY) {

    // return this.run(() -> moveMotorsWithIMU(leftY,rightY));
    // }

    public void setLeftPower(double leftY) {
        leftPower = leftY;
        motorLeft.set(leftY);
    }

    public void setRightPower(double rightY) {
        rightPower = rightY;
        motorRight.set(rightY);
    }

    // public void moveMotorsWithIMU(double leftY, double rightY) {
    // double roll = imu.getRoll();

    // if (roll != 0) {
    // // If the roll is not 0, adjust the power of the motors to correct the roll.
    // // This is a simple example and might need to be adjusted based on your
    // specific requirements.
    // double adjustment = roll * 0.01; // You might need to adjust this factor

    // leftY -= adjustment;
    // rightY += adjustment;
    // }

    // // Move the motors
    // setPower(leftY, rightY);
    // }

    public double getLeftPosition() {
        return m_leftEncoder.getPosition();
    }

    public double getRightPosition() {
        return m_rightEncoder.getPosition();
    }

    public boolean isLeftHome() {
        if (!leftHome.get()){
            m_leftEncoder.setPosition(0.0);
        }
        return leftHome.get();
    }

    public boolean isRightHome() {
        if(!rightHome.get()) {
            m_rightEncoder.setPosition(0.0);
        }
        return rightHome.get();
    }

}
