package frc.team1126.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team1126.RobotContainer;
import frc.team1126.subsystems.Arm;

public class ArmToGround extends Command  {
   private Arm arm;


   public ArmToGround(Arm arm) {
    addRequirements(RobotContainer.arm);
    this.arm = arm;
   }

   @Override
   public void execute(){
       
       while(!arm.getHomeLimit()) {
        arm.moveArm(-.10);
       }
    
    }

    @Override
    public void end(boolean interupted){
        arm.moveArm(0);
        arm.zeroPigeon();
    }

    @Override
    public boolean isFinished() {
      return false;
    }

}
