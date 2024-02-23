package frc.team1126.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team1126.Constants.ArmConstants;

public class Arm extends SubsystemBase {

    private CANSparkMax m_armLeftMotor;
    private CANSparkMax m_armRightMotor;

    private RelativeEncoder m_armLeftEncoder;
    private RelativeEncoder m_armRightEncoder;

    private Pigeon2 m_armPigeon;
    public DigitalInput m_homeLimit;
    private final SparkPIDController m_sparkPIDController;
    private final PIDController m_pidController = new PIDController(0.028, 0.028, 0.003); 
    //0.03, 0.0, 0.00 // 0.028, 0.028, 0.0022
    private double m_targetAngle;
    private double m_pidOutput;

    private double m_power = 0;

    public Arm() {
        m_armLeftMotor = new CANSparkMax(ArmConstants.MASTER_ID, CANSparkLowLevel.MotorType.kBrushless);
        m_armRightMotor = new CANSparkMax(ArmConstants.SLAVE_ID, CANSparkLowLevel.MotorType.kBrushless);
        m_armPigeon = new Pigeon2(ArmConstants.ARM_PIGEON_ID);
        initPigeon();
        m_homeLimit = new DigitalInput(ArmConstants.ARM_LIMIT_SWITCH_ID);

        m_armLeftMotor.restoreFactoryDefaults();
        m_armRightMotor.restoreFactoryDefaults();
        // m_armLeftMotor.follow(m_armRightMotor);
        m_armRightMotor.setInverted(false);
        m_armLeftMotor.setInverted(true);
        m_armLeftMotor.setIdleMode(IdleMode.kBrake);
        m_armRightMotor.setIdleMode(IdleMode.kBrake);

        m_armLeftEncoder = m_armLeftMotor.getEncoder();
        m_armRightEncoder = m_armRightMotor.getEncoder();
        m_sparkPIDController = m_armRightMotor.getPIDController();
        m_sparkPIDController.setFeedbackDevice(m_armRightEncoder);
        set(PIDF.PROPORTION, PIDF.INTEGRAL, PIDF.DERIVATIVE,
                PIDF.FEEDFORWARD, PIDF.INTEGRAL_ZONE);

               
    }

    private void initPigeon() {
        // Factory default the Pigeon.
        var toApply = new Pigeon2Configuration();
        // var mountPose = toApply.MountPose;
        toApply.MountPose.MountPosePitch = 0;
        // toApply.MountPose.MountPoseRoll = 0;
        // toApply.MountPose.MountPoseYaw = -90;
        /*
         * User can change the configs if they want, or leave it empty for
         * factory-default
         */

         m_armPigeon.getConfigurator().apply(toApply);

        m_armPigeon.getPitch().setUpdateFrequency(1000);
        m_armPigeon.setYaw(0, .1);

        m_armPigeon.getYaw().setUpdateFrequency(100);
        m_armPigeon.getYaw().waitForUpdate(.1);
    }

    public static class PIDF {

        /**
         * Feedforward constant for PID loop
         */
        public static final double FEEDFORWARD = 0.01;
        /**
         * Proportion constant for PID loop
         */
        public static final double PROPORTION = 0.05;
        /**
         * Integral constant for PID loop
         */
        public static final double INTEGRAL = 0.0;
        /**
         * Derivative constant for PID loop
         */
        public static final double DERIVATIVE = 0.0;
        /**
         * Integral zone constant for PID loop
         */
        public static final double INTEGRAL_ZONE = 0.0;
    }

    public void set(double p, double i, double d, double f, double iz) {
        m_sparkPIDController.setP(p);
        m_sparkPIDController.setI(i);
        m_sparkPIDController.setD(d);
        m_sparkPIDController.setFF(f);
        m_sparkPIDController.setIZone(iz);
    }

    public void runPID(double targetPosition) {
        m_sparkPIDController.setReference(targetPosition, CANSparkMax.ControlType.kPosition);
    }
    public void runPigeonPID(double targetAngle) {
        double currentAngle = getPitch();
        if (currentAngle <0){
            currentAngle = 0;
        }
        double error =  currentAngle -targetAngle;
        m_targetAngle = error;
        double pidOutput = m_pidController.calculate(error);
        // double feedForward = PIDF.FEEDFORWARD * targetAngle; // calculate feedforward term
        double totalOutput = pidOutput + .094762*Math.cos(m_armPigeon.getPitch().getValueAsDouble()); //+ feedForward; // add feedforward to PID output

        m_pidOutput = totalOutput;
        m_armRightMotor.set(totalOutput);
        m_armLeftMotor.set(totalOutput);
    }

    public void runAmpPID(double targetAngle) {
        double currentAngle = getPitch();
        if (currentAngle <0){
            currentAngle = 0;
        }

        try (PIDController pidController = new PIDController(0.01, 0.0, 0.000)) {
            double error =  currentAngle -targetAngle;
            m_targetAngle = error;
            double pidOutput = pidController.calculate(error);
            // double feedForward = PIDF.FEEDFORWARD * targetAngle; // calculate feedforward term
            double totalOutput = pidOutput + .094762*Math.cos(m_armPigeon.getPitch().getValueAsDouble()); //+ feedForward; // add feedforward to PID output

            m_pidOutput = totalOutput;
            m_armRightMotor.set(totalOutput);
            m_armLeftMotor.set(totalOutput);
        }
    }
    public void runClimbPID(double targetAngle) {

        try (PIDController pidController = new PIDController(0.015, 0.0, 0.000)) {
            double currentAngle = getPitch();
            if (currentAngle <0){
                currentAngle = 0;
            }
            double error =  currentAngle -targetAngle;
            m_targetAngle = error;
            double pidOutput = pidController.calculate(error);
            double totalOutput = pidOutput ;

            m_pidOutput = totalOutput;
            m_armRightMotor.set(totalOutput);
            m_armLeftMotor.set(totalOutput);
        }
    }

    public double getAngle() {
        return m_armRightEncoder.getPosition() * 360;
    }
    public Command setAngleCommand(double angle) {
        return this.run(() -> runPigeonPID(angle));
    }

    @Override
    public void periodic() {

        SmartDashboard.putNumber("arm Pitch", getPitch());
        SmartDashboard.putNumber("Arm Power", m_power);
        SmartDashboard.putNumber("Arm Left", m_armLeftMotor.get());
        SmartDashboard.putNumber("Arm Right", m_armRightMotor.get());
        // SmartDashboard.putNumber("Arm Angle",getAngle());
        // SmartDashboard.putNumber("Target Angle", m_targetAngle);
        SmartDashboard.putNumber("Arm left speed", m_armLeftEncoder.getVelocity());
        SmartDashboard.putNumber("Arm right speed", m_armRightEncoder.getVelocity());
        SmartDashboard.putBoolean("Home Limit", m_homeLimit.get()); // getHomeLimit());\
        SmartDashboard.putNumber("PID Output", m_pidOutput);
    }

    public void changeAngle(double liftPower) {
        m_armRightMotor.set(liftPower);
        m_armLeftMotor.set(liftPower);
    }

    public Command runManual(DoubleSupplier supplier) {
        double power = supplier.getAsDouble();
        return run(() -> {
            changeAngle(power);
        });
    }

    public void moveArm(double power) {
        m_power = power;
        m_armLeftMotor.set(m_power);
        m_armRightMotor.set(m_power);
    }

    public double getPitch() {
        return m_armPigeon.getRoll().getValue();
    }
    public void zeroPigeon() {
        m_armPigeon.reset();
    }

    public boolean getHomeLimit() {
        return m_homeLimit.get();
    }
    public Command setAngle(double degrees) {
        m_targetAngle = degrees;
        return run(() -> {
            runPID(degrees);
        });
    }

}