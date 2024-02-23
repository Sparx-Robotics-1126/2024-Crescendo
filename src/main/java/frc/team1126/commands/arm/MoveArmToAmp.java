package frc.team1126.commands.arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team1126.RobotContainer;
import frc.team1126.subsystems.Arm;
import frc.team1126.subsystems.sensors.Limelight;

public class MoveArmToAmp extends Command {
    private double m_targetAngle;
    private final Arm m_arm;
    private final Limelight m_Limelight = Limelight.getInstance();

    public MoveArmToAmp(Arm arm) { // -3
        addRequirements(RobotContainer.m_arm, RobotContainer.m_limeLight);
        m_arm = arm;

        m_targetAngle = 53;
    }

    @Override
    public void execute() {
        // m_targetAngle = m_Limelight.getShootingAngle();
        // System.out.println("This angle " + m_targetAngle );
        double currentPitch = m_arm.getPitch();
        // double target = m_targetAngle;
        SmartDashboard.putNumber("Target Angle", m_targetAngle);
        if (currentPitch < m_targetAngle && m_arm.getPitch() < 85) {
            m_arm.runAmpPID(m_targetAngle); // positive power to move up
        } else if (currentPitch > m_targetAngle && !m_arm.m_homeLimit.get()) {
            m_arm.runAmpPID(m_targetAngle); // negative power to move down
        }
    }

    @Override
    public void end(boolean interupted) {
        m_arm.moveArm(0);
    }

    @Override
    public boolean isFinished() {

        // if (m_arm.getPitch() >= m_targetAngle) {
        // return true;
        // }
        return false;
    }

}