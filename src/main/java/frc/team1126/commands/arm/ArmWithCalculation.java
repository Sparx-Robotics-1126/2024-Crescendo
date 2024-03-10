package frc.team1126.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team1126.RobotContainer;
import frc.team1126.subsystems.Arm;

public class ArmWithCalculation extends Command {
    private Arm m_arm;

    public ArmWithCalculation(Arm arm) {
        addRequirements(RobotContainer.m_arm);
        m_arm = arm;
    }

    @Override
    public void execute() {
    
        m_arm.moveArmToAngle(m_arm.calcAngle());
    }


    
}
