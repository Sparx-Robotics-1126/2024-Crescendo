package frc.team1126.commands.Storage;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team1126.RobotContainer;
import frc.team1126.subsystems.Storage;

public class EjectNote extends Command {

    private Storage m_storage;
    private double m_speed;
    
    public EjectNote(Storage storage) {
       // addRequirements(RobotContainer.m_storage);
        m_storage = storage;
        m_speed = -1;

    }

    @Override
    public void execute(){


        m_storage.setStorageWheels(m_speed);
    }

       @Override
   public void end(boolean interrupted) {
    m_storage.resetNote();
        m_storage.setStorageWheels(0.0);
   }

   
   @Override
   public boolean isFinished() {

    m_storage.resetNote();
     return false;
   }
}
