package frc.team1126.commands.arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team1126.RobotContainer;
import frc.team1126.subsystems.Arm;

public class MoveArmForClimb extends Command {
    private final double m_targetAngle;
    private final Arm m_arm;

    public MoveArmForClimb(Arm arm) { // -3
        addRequirements(RobotContainer.m_arm);
        m_arm = arm;
        m_targetAngle = 80;
    }

    
    @Override
    public void execute() {
        double currentPitch = m_arm.getPitch();

        if (currentPitch < m_targetAngle && m_arm.getPitch() <85) {
            m_arm.moveArmToAngle(m_targetAngle); // positive power to move up
        } 
        // else if (currentPitch > m_targetAngle && !m_arm.m_homeLimit.get()) {
        //     m_arm.runClimbPID(m_targetAngle); // negative power to move down
        // }
        //
    }

    @Override
    public void end(boolean interupted) {
        m_arm.moveArm(0);
    }

    @Override
    public boolean isFinished() {

        // if (m_arm.getPitch() >= m_targetAngle) {
        //     return true;
        // }
        return false;
    }


}