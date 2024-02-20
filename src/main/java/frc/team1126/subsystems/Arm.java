package frc.team1126.subsystems;

import java.util.function.DoubleSupplier;

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

    private CANSparkMax armLeftMotor;
    private CANSparkMax armRightMotor;

    private RelativeEncoder armLeftEncoder;
    private RelativeEncoder armRightEncoder;

    private Pigeon2 armPigeon;
    private DigitalInput homeLimit;
    private final SparkPIDController sparkPIDController;
    private final PIDController pidController = new PIDController(0.05, 0.0, 0.0);
    private double targetAngle;
    

    private double power = 0;

    public Arm() {
        armLeftMotor = new CANSparkMax(ArmConstants.MASTER_ID, CANSparkLowLevel.MotorType.kBrushless);
        armRightMotor = new CANSparkMax(ArmConstants.SLAVE_ID, CANSparkLowLevel.MotorType.kBrushless);
        armPigeon = new Pigeon2(ArmConstants.ARM_PIGEON_ID);
        homeLimit = new DigitalInput(ArmConstants.ARM_LIMIT_SWITCH_ID);

        armLeftMotor.restoreFactoryDefaults();
        armRightMotor.restoreFactoryDefaults();
        armLeftMotor.follow(armRightMotor);
        armLeftMotor.setInverted(false);
        armLeftMotor.setIdleMode(IdleMode.kBrake);
        armRightMotor.setIdleMode(IdleMode.kBrake);

        armLeftEncoder = armLeftMotor.getEncoder();
        armRightEncoder = armRightMotor.getEncoder();
        sparkPIDController = armRightMotor.getPIDController();
        sparkPIDController.setFeedbackDevice(armRightEncoder);
        set(PIDF.PROPORTION, PIDF.INTEGRAL, PIDF.DERIVATIVE,
                PIDF.FEEDFORWARD, PIDF.INTEGRAL_ZONE);
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
        sparkPIDController.setP(p);
        sparkPIDController.setI(i);
        sparkPIDController.setD(d);
        sparkPIDController.setFF(f);
        sparkPIDController.setIZone(iz);
    }

    public void runPID(double targetPosition) {
        sparkPIDController.setReference(targetPosition, CANSparkMax.ControlType.kPosition);
    }
    public void runPigeonPID(double targetAngle) {
        double currentAngle = getPitch();
        double error = targetAngle - currentAngle;
        double pidOutput = pidController.calculate(error);
        double feedForward = PIDF.FEEDFORWARD * targetAngle; // calculate feedforward term
        double totalOutput = pidOutput + feedForward; // add feedforward to PID output
        armRightMotor.set(totalOutput);
    }

    public double getAngle() {
        return armRightEncoder.getPosition() * 360;
    }
    public Command setAngleCommand(double angle) {
        return this.run(() -> runPigeonPID(angle));
    }

    @Override
    public void periodic() {

        SmartDashboard.putNumber("arm Pitch", armPigeon.getPitch().getValue());
        SmartDashboard.putNumber("Arm Power", power);
        SmartDashboard.putNumber("Arm Left", armLeftMotor.get());
        SmartDashboard.putNumber("Arm Right", armRightMotor.get());
        SmartDashboard.putNumber("Arm Angle",getAngle());
        SmartDashboard.putNumber("Arm left speed", armLeftEncoder.getVelocity());
        SmartDashboard.putNumber("Arm right speed", armRightEncoder.getVelocity());
        SmartDashboard.putBoolean("Home Limit", getHomeLimit());
    }

    public void changeAngle(double liftPower) {
        armRightMotor.set(liftPower);
    }

    public Command runManual(DoubleSupplier supplier) {
        double power = supplier.getAsDouble();
        return run(() -> {
            changeAngle(power);
        });
    }

    public void moveArm(double power) {
        this.power = -power;
        armLeftMotor.set(power);
        armRightMotor.set(power);
    }

    public double getPitch() {
        return armPigeon.getPitch().getValue();
    }
    public void zeroPigeon() {
        armPigeon.reset();
    }

    public boolean getHomeLimit() {
        return !homeLimit.get();
    }
    public Command setAngle(double degrees) {
        targetAngle = degrees;
        return run(() -> {
            runPID(degrees);
        });
    }

}