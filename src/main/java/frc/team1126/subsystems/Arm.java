package frc.team1126.subsystems;

import java.util.EnumMap;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team1126.Constants;
import frc.team1126.Constants.ArmConstants;
import frc.team1126.Constants.GeneralConstants;
import frc.team1126.lib.properties.feedforward.ArmFeedForwardProperty;
import frc.team1126.lib.properties.pid.PidProperty;
import frc.team1126.lib.properties.pid.RevPidPropertyBuilder;
import frc.team1126.lib.properties.pid.WpiProfiledPidPropertyBuilder;
import frc.team1126.subsystems.sensors.Limelight;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.ControlType;

public class Arm extends SubsystemBase {

    private CANSparkMax m_armLeftMotor;
    private CANSparkMax m_armRightMotor;

    private RelativeEncoder m_armLeftEncoder;
    private RelativeEncoder m_armRightEncoder;

    private Limelight m_limelight = Limelight.getInstance();

    private Pigeon2 m_armPigeon;
    public DigitalInput m_homeLimit;
    // private final SparkPIDController m_sparkPIDController;
    private final PIDController m_pidController = new PIDController(0.028, 0.025, 0.002); 
    //0.03, 0.0, 0.00 // 0.028, 0.028, 0.0022
    private double m_targetAngle;
    private double m_pidOutput;

    private double m_power = 0;
//  private final RelativeEncoder m_armMotorEncoder;
    private final AbsoluteEncoder m_amrAbsEncoder;
    private final SparkPIDController m_sparkPidController;
    private final ProfiledPIDController m_profilePID;
    private final PidProperty m_sparkPidProperties;
     private final PidProperty m_profilePidProperties;
    private final ArmFeedForwardProperty m_wpiFeedForward;
    private double m_armGoalAngle = Double.MIN_VALUE;
    private static final double ARM_MAX_ANGLE = 90;
    //   private final DutyCycleEncoder armAbsEncoder = new DutyCycleEncoder(0);
//   double m_speed = 0.0;
//   EnumMap<armPositions, Double> mapAbs = new EnumMap<>(armPositions.class);
//   private final PIDController m_AbsPidController = new PIDController(ArmConstants.kArmP, ArmConstants.kArmI,
    //   ArmConstants.kArmD);

    public Arm() {
        m_armLeftMotor = new CANSparkMax(ArmConstants.MASTER_ID, CANSparkLowLevel.MotorType.kBrushless);
        m_armRightMotor = new CANSparkMax(ArmConstants.SLAVE_ID, CANSparkLowLevel.MotorType.kBrushless);
        m_armLeftMotor.restoreFactoryDefaults();
        m_armRightMotor.restoreFactoryDefaults();
        m_armLeftMotor.setIdleMode(IdleMode.kBrake);
        m_armRightMotor.setIdleMode(IdleMode.kBrake);
        m_armRightMotor.setSmartCurrentLimit(60);
        m_armLeftMotor.setSmartCurrentLimit(60);
        m_armRightMotor.setInverted(false);
       
        m_armLeftMotor.follow(m_armRightMotor, true);

        m_armPigeon = new Pigeon2(ArmConstants.ARM_PIGEON_ID);
        initPigeon();
        m_homeLimit = new DigitalInput(ArmConstants.ARM_LIMIT_SWITCH_ID);

        m_armRightMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus5, 20);
        m_armRightMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus6, 20);


        m_armLeftEncoder = m_armLeftMotor.getEncoder();
        m_armRightEncoder = m_armRightMotor.getEncoder();
        m_armRightEncoder.setPositionConversionFactor(360.0 / 100);
        m_armRightEncoder.setVelocityConversionFactor(360.0 / 100 / 60);

        m_amrAbsEncoder = m_armRightMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        m_amrAbsEncoder.setPositionConversionFactor(360.0);
        m_amrAbsEncoder.setVelocityConversionFactor(360.0 / 60);
        m_amrAbsEncoder.setInverted(false);
        m_amrAbsEncoder.setZeroOffset(277.11 + 2);

        m_sparkPidController = m_armRightMotor.getPIDController();
        m_sparkPidController.setFeedbackDevice(m_armRightEncoder);

          m_sparkPidController.setPositionPIDWrappingEnabled(true);
        m_sparkPidController.setPositionPIDWrappingMinInput(0);
        m_sparkPidController.setPositionPIDWrappingMaxInput(360);

        m_sparkPidProperties = new RevPidPropertyBuilder("Arm Pivot",true, m_sparkPidController, 0)
            .addP(.03)
            .addI(0)
            .addD(0)
            .build();
        m_profilePID = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0));
        m_profilePID.enableContinuousInput(0, 360);
        m_profilePidProperties = new WpiProfiledPidPropertyBuilder("Arm Profile PID", true, m_profilePID)
            .addMaxVelocity(120)
            .addMaxAcceleration(120)
            .build();
        m_wpiFeedForward = new ArmFeedForwardProperty("Arm Pivot Profile ff", true)
            .addKs(0)
            .addKff(2.2)
            .addKg(1.2);
    
            m_armLeftMotor.burnFlash();
            m_armRightMotor.burnFlash();
            m_armLeftEncoder.setPosition(m_amrAbsEncoder.getPosition());
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

private double getP(){
    return SmartDashboard.getNumber("Arm P",.14);
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

    // public void set(double p, double i, double d, double f, double iz) {
    //     m_sparkPIDController.setP(p);
    //     m_sparkPIDController.setI(i);
    //     m_sparkPIDController.setD(d);
    //     m_sparkPIDController.setFF(f);
    //     m_sparkPIDController.setIZone(iz);
    // }
    public double calcAngle(){
        if(m_limelight.hasSpeakerTarget()) {
            var dist = m_limelight.getDistance();
            return  69- Math.toDegrees((Math.atan(50/(dist+35)))); // change the 59 or the 35 to tune
        }
        return GeneralConstants.CLOSE_SPEAKER_ANGLE;
        
    }
    private void resetPidController() {
        m_profilePID.reset(getAngle(), getEncoderVel());
    }
    public double getEncoderVel() {
        return m_armLeftEncoder.getVelocity();
    }
    public void moveArmToAngle(double goalAngle) {
        if (Math.abs(m_armGoalAngle - goalAngle) > 2) {
            //resetPidController();
        }

        m_armGoalAngle = goalAngle;
        double currentAngle = getAngle();
        // if (currentAngle < ARM_MAX_ANGLE || MathUtil.inputModulus(goalAngle, -180, 180) < MathUtil.inputModulus(currentAngle, -180, 180)) {
            if (currentAngle < ARM_MAX_ANGLE ) {
            m_profilePID.calculate(currentAngle, goalAngle);
            TrapezoidProfile.State setpoint = m_profilePID.getSetpoint();
            double feedForwardVolts = m_wpiFeedForward.calculate(
                Units.degreesToRadians(currentAngle),
                Units.degreesToRadians(setpoint.velocity));


            m_sparkPidController.setReference(setpoint.position, CANSparkMax.ControlType.kPosition, 0, feedForwardVolts);
            SmartDashboard.putNumber("feedForwardVolts", feedForwardVolts);
        }
        else {
            stopArmMotor();
        }
    }

    public void stopArmMotor() {
        m_armRightMotor.set(0);
        m_armGoalAngle = Double.MIN_VALUE;
    }

    public void runPID(double targetPosition) {
        // m_sparkPIDController.setReference(targetPosition, CANSparkMax.ControlType.kPosition);
    }
public void runPigeonPID() {

    runPigeonPID(calcAngle());
}

    public void runPigeonPID(double targetAngle) {
        double currentAngle = getPitch();
        if (currentAngle <0){
            currentAngle = 0;
        }
        double error =  currentAngle -targetAngle;
        m_targetAngle = error;
        double pidOutput = m_pidController.calculate(error);
        double feedForward = PIDF.FEEDFORWARD * targetAngle; // calculate feedforward term
        double totalOutput = pidOutput + .023913*Math.cos(m_armPigeon.getPitch().getValueAsDouble()); //+ feedForward; // add feedforward to PID output

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
            double feedForward = PIDF.FEEDFORWARD * targetAngle; // calculate feedforward term
            double totalOutput = pidOutput + .094762*Math.cos(m_armPigeon.getPitch().getValueAsDouble()); //+ feedForward; // add feedforward to PID output

            m_pidOutput = totalOutput;
            m_armRightMotor.set(totalOutput);
            // m_armLeftMotor.set(totalOutput);
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
            //  m_armLeftMotor.set(totalOutput);
        }
    }

    public double getAngle() {
        return m_armRightEncoder.getPosition() ;
    }
    public Command setAngleCommand(double angle) {
        return this.run(() -> runPigeonPID(angle));
    }

    @Override
    public void periodic() {

        // m_sparkPidController.setP(SmartDashboard.getNumber("Arm P", 0));
        // m_sparkPidController.setI(SmartDashboard.getNumber("Arm I", 0));
        // m_sparkPidController.setD(SmartDashboard.getNumber("Arm D", 0));
        
        SmartDashboard.putNumber("arm Pitch", getPitch());
        SmartDashboard.putNumber("Arm Power", m_power);
        SmartDashboard.putNumber("Arm Left", m_armLeftMotor.get());
        SmartDashboard.putNumber("Arm Right", m_armRightMotor.get());
        // SmartDashboard.putNumber("Arm Angle",getAngle());
        // SmartDashboard.putNumber("Target Angle", m_targetAngle);
        SmartDashboard.putNumber("Arm left speed", m_armLeftEncoder.getVelocity());
        SmartDashboard.putNumber("Arm right speed", m_armRightEncoder.getVelocity());
        SmartDashboard.putNumber("Arm Position", m_armRightEncoder.getPosition());
        SmartDashboard.putBoolean("Home Limit", m_homeLimit.get()); // getHomeLimit());\
        SmartDashboard.putNumber("PID Output", m_pidOutput);
        SmartDashboard.putNumber("Target ANgle", calcAngle());
        // SmartDashboard.putNumber("Arm P",m_sparkPidController.getP());
        // SmartDashboard.putNumber("Arm I",m_sparkPidController.getI());
        // SmartDashboard.putNumber("Arm D",m_sparkPidController.getD());
        // SmartDashboard.putNumber("Arm FF",m_sparkPidController.getFF());
        m_sparkPidProperties.updateIfChanged();
        m_wpiFeedForward.updateIfChanged();
        m_profilePidProperties.updateIfChanged();
    }

    public void changeAngle(double liftPower) {
        m_armRightMotor.set(liftPower);
        //  m_armLeftMotor.set(liftPower);
    }

    public Command runManual(DoubleSupplier supplier) {
        double power = supplier.getAsDouble();
        return run(() -> {
            changeAngle(power);
        });
    }

    public void moveArm(double power) {
        m_power = power;
        //  m_armLeftMotor.set(m_power);
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

    public boolean isArmAtGoal() {
        double error = m_armGoalAngle - getAngle();
        return Math.abs(error) < 1.7;
    }

}