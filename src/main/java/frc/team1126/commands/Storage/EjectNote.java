package frc.team1126.commands.Storage;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team1126.RobotContainer;
import frc.team1126.subsystems.Storage;

public class EjectNote extends Command {

    private Storage storage;
    private double speed;
    
    public EjectNote(Storage storage) {
        addRequirements(RobotContainer.storage);
        this.storage = storage;
        this.speed = -1;

    }

    @Override
    public void execute(){
 
    storage.setHasNote();


        this.storage.setStorageWheels(speed);
    }

       @Override
   public void end(boolean interrupted) {
       storage.setStorageWheels(0.0);
   }

   
   @Override
   public boolean isFinished() {
     return false;
   }
}
