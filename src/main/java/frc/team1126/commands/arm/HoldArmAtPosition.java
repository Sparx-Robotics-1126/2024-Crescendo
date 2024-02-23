package frc.team1126.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team1126.RobotContainer;
import frc.team1126.subsystems.Arm;

public class HoldArmAtPosition extends Command {
    Arm m_arm;
    double m_angle;

    public HoldArmAtPosition(Arm arm) {
        addRequirements(RobotContainer.m_arm);
        m_arm = arm;
        m_angle = m_arm.getPitch();
    }

     @Override
    public void execute(){
       m_arm.runPigeonPID(m_arm.getPitch());

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
