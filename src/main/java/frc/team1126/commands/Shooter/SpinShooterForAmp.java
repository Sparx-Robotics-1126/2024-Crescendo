package frc.team1126.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team1126.RobotContainer;
import frc.team1126.subsystems.Shooter;

public class SpinShooterForAmp extends Command {
    

    private Shooter m_shooter;

    public SpinShooterForAmp(Shooter shooter) {

        addRequirements(RobotContainer.m_shooter);
        m_shooter = shooter;

    }

    @Override
    public void execute(){
        m_shooter.setShooterSpeed(.3);
    }

       @Override
   public void end(boolean interrupted) {
        m_shooter.setShooterSpeed(0.0);
   }

   
   @Override
   public boolean isFinished() {
     return false;
   }
}
