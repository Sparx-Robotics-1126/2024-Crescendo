// package frc.team1126.commands.Storage;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.team1126.RobotContainer;
// import frc.team1126.subsystems.Storage;

// public class CheckForNote extends Command {
//    private Storage storage;


//    public CheckForNote(Storage storage) {
//     addRequirements(RobotContainer.storage);
//     this.storage = storage;
//    }

//    @Override
//    public void execute() {
//     while(storage.hasNote()) {
//        storage.setStorageWheels(0.5);
//     } 
//     storage.setHasNote();

//    }

//    @Override
//    public void end(boolean interrupted) {
//        storage.setStorageWheels(0.0);
//    }

   
//    @Override
//    public boolean isFinished() {
//      return false;
//    }
// }
