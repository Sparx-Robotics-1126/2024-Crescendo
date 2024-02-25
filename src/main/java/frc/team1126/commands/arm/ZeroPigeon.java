package frc.team1126.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team1126.RobotContainer;
import frc.team1126.subsystems.Arm;

public class ZeroPigeon extends Command  {
   private Arm m_arm;


   public ZeroPigeon(Arm arm) {
    addRequirements(RobotContainer.m_arm);
    m_arm = arm;
   }

   @Override
   public void execute(){
      m_arm.zeroPigeon();
    
    }

    @Override
    public void end(boolean interupted){
        m_arm.zeroPigeon();
    }

    @Override
    public boolean isFinished() {
      return false;
    }

}
