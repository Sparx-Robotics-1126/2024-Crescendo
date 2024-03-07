package frc.team1126.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team1126.Constants.ShooterConstants;
import frc.team1126.subsystems.Arm.PIDF;

public class Shooter extends SubsystemBase {

    private CANSparkFlex m_shooterMotor;
    private RelativeEncoder m_shooterEncoder;
    private PIDController m_pidController;
    private final SparkPIDController m_sparkPIDController;
    


    private double shooterSpeed;

    public Shooter() {
        m_shooterMotor = new CANSparkFlex(ShooterConstants.SHOOTER_ID, CANSparkLowLevel.MotorType.kBrushless);
        m_shooterEncoder = m_shooterMotor.getEncoder();
        m_sparkPIDController =  m_shooterMotor.getPIDController();
        m_sparkPIDController.setFeedbackDevice(m_shooterEncoder);
        set(PIDF.PROPORTION, PIDF.INTEGRAL, PIDF.DERIVATIVE,
                PIDF.FEEDFORWARD, PIDF.INTEGRAL_ZONE);

    }

    public void set(double p, double i, double d, double f, double iz) {
        m_sparkPIDController.setP(p);
        m_sparkPIDController.setI(i);
        m_sparkPIDController.setD(d);
        m_sparkPIDController.setFF(f);
        m_sparkPIDController.setIZone(iz);
    }
    public void setShooterSpeed(double speed) {
        shooterSpeed = speed;
        m_shooterMotor.set(speed);
    }
   
    //check to see if this works
    public void setShooterPID(double targetSpeed) {
       double currentShooterSpeed = getShooterSpeed();

       if(currentShooterSpeed < 0) {
        currentShooterSpeed = 0;
       }
      
       double error = currentShooterSpeed - targetSpeed;
       double pidOutput = m_pidController.calculate(error);

       m_shooterMotor.set(pidOutput);

    }
    //ñ

    public double getShooterSpeed() {
        return m_shooterMotor.get();
    }

    public boolean isMotorUpToSpeed() {
        double currentSpeed = m_shooterMotor.get();
        return currentSpeed >= shooterSpeed;
    }

}
