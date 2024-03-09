package frc.team1126.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team1126.RobotContainer;
import frc.team1126.subsystems.Shooter;
import frc.team1126.subsystems.Storage;


public class SpinShooterForAmp extends Command {
    

    private Shooter m_shooter;
    private Storage m_storage;


    public SpinShooterForAmp(Shooter shooter, Storage storage) {

        addRequirements(RobotContainer.m_shooter, RobotContainer.m_storage);
        m_shooter = shooter;
        m_storage = storage;
    }

    @Override
    public void execute(){
        m_shooter.setShooterSpeed(.3);
        if(m_storage.hasNote()) {
           m_storage.resetNote();
        }
        
    }

       @Override
   public void end(boolean interrupted) {
        m_shooter.setShooterSpeed(0.0);
        m_storage.resetNote();
   }

   
   @Override
   public boolean isFinished() {
    m_storage.resetNote();
    return false;
     
   }
   
}
