package frc.team1126.commands.arm;

import java.util.function.DoubleSupplier;

import javax.swing.text.StyleContext.SmallAttributeSet;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team1126.RobotContainer;
import frc.team1126.subsystems.Arm;
import frc.team1126.subsystems.sensors.Limelight;

public class MoveArmWithLimelight extends Command {
    private final Arm m_arm;
    private final Limelight m_Limelight;

    public MoveArmWithLimelight(Arm arm, Limelight limelight) { // -3
        addRequirements(RobotContainer.m_arm, RobotContainer.m_limeLight);
        m_arm = arm;
        m_Limelight = limelight;
    }

    @Override
    public void execute() {
        double currentPitch = m_arm.getPitch();
        double target = m_Limelight.calculateTargetAngle();
            // SmartDashboard.putNumber("Target Angle", target);
            if (currentPitch < target && m_arm.getPitch() < 85) {
                m_arm.runPigeonPID(target); // positive power to move up
            } else if (currentPitch > target && !m_arm.m_homeLimit.get()) {
                m_arm.runPigeonPID(target); // negative power to move down
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