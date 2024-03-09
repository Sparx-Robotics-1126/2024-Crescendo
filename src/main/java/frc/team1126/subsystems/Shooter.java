package frc.team1126.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkBase;
// import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team1126.Constants;
import frc.team1126.Constants.ShooterConstants;
import frc.team1126.lib.properties.pid.PidProperty;
import frc.team1126.lib.properties.pid.RevPidPropertyBuilder;
//import frc.team1126.lib.properties.pid.RevPidPropertyBuilder;
import frc.team1126.subsystems.Arm.PIDF;

public class Shooter extends SubsystemBase {

    private CANSparkFlex m_shooterMotor;
    private RelativeEncoder m_shooterEncoder;
    private PIDController m_pidController = new PIDController(1, 0, 1.2);
    private final SparkPIDController m_sparkPIDController;
    private final PidProperty m_pidProperties;
    


    private double m_shooterSpeed;

    public Shooter() {
        m_shooterMotor = new CANSparkFlex(ShooterConstants.SHOOTER_ID, CANSparkLowLevel.MotorType.kBrushless);
        m_shooterEncoder = m_shooterMotor.getEncoder();
        m_sparkPIDController =  m_shooterMotor.getPIDController();
        //m_pidController = m_shooterMotorLeader.getPIDController();
        m_pidProperties = new RevPidPropertyBuilder("Shooter", Constants.DEFAULT_CONSTANT_PROPERTIES, m_sparkPIDController, 0)
            .addP(0.003) //1.5e-4
            .addI(0.0)
            .addD(0.14)
            .addFF(0.000) //0.000185
            .build();

    }


    @Override
    public void periodic() {
        SmartDashboard.putNumber("PID shooter speed", m_shooterSpeed);
        SmartDashboard.putNumber("Shooter Power", m_shooterEncoder.getVelocity());
    }
    public void setShooterSpeed(double speed) {
        m_shooterSpeed = speed;
        m_shooterMotor.set(speed);
    }
   
    //check to see if this works
    public void setShooterPID(double targetSpeed) {
        
        m_sparkPIDController.setReference(targetSpeed, CANSparkBase.ControlType.kVelocity);
        m_shooterSpeed = targetSpeed;
   
   
        //    double currentShooterSpeed = getShooterSpeed();

    //    if(currentShooterSpeed < 0) {
    //     currentShooterSpeed = 0;
    //    }
      
    //    double error = currentShooterSpeed - targetSpeed;
    //    double pidOutput = m_pidController.calculate(error);

    //    m_shooterMotor.set(pidOutput);



    }
    //ñ

    public double getShooterSpeed() {
        return m_shooterMotor.get();
    }

    public boolean isMotorUpToSpeed() {
        double currentSpeed = m_shooterMotor.get();
        return currentSpeed >= m_shooterSpeed;
    }

}
