package frc.team1126.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team1126.RobotContainer;
import frc.team1126.subsystems.Arm;

public class ArmToGround extends Command  {
   private Arm m_arm;


   public ArmToGround(Arm arm) {
    addRequirements(RobotContainer.m_arm);
    m_arm = arm;
   }

   @Override
   public void execute(){
       
       if(!m_arm.getHomeLimit()) {
        m_arm.moveArm(-.20);
       }
    
    }

    @Override
    public void end(boolean interupted){
        m_arm.moveArm(0);
        m_arm.zeroPigeon();
    }

    @Override
    public boolean isFinished() {
      return false;
    }

}
