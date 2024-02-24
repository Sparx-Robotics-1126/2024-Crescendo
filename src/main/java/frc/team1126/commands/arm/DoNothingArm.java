package frc.team1126.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team1126.RobotContainer;
import frc.team1126.subsystems.Arm;

public class DoNothingArm extends Command{
    
    private Arm m_arm;

    public DoNothingArm(Arm arm) {
        addRequirements(RobotContainer.m_arm);
        m_arm = arm;
    }

    @Override
    public void execute() {
        m_arm.moveArm(0.2);
    }
   
    @Override
    public void end(boolean interupted){
        m_arm.moveArm(0);
    }

    @Override
    public boolean isFinished() {
      return false;
    }
}










//Bingus
