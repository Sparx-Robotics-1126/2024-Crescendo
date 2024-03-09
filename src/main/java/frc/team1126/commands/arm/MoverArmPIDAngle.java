package frc.team1126.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team1126.subsystems.Arm;

public class MoverArmPIDAngle extends Command {
    private final Arm m_arm;
    private double m_targetAngle;

    public MoverArmPIDAngle(Arm arm, double targetAngle) {
        m_arm = arm;
        m_targetAngle = targetAngle;
        addRequirements(m_arm);

    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        super.end(interrupted);
    }

    @Override
    public void execute() {
        m_arm.moveArmToAngle(m_targetAngle);
    }

    @Override
    public void initialize() {
    }

    // @Override
    // public boolean isFinished() {
    //     if (m_arm.isArmAtGoal()) {
    //         return true;
    //     }
    //     return false;
    // }

}
