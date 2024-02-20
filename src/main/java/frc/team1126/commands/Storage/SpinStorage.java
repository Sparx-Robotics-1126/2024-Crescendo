package frc.team1126.commands.Storage;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team1126.RobotContainer;
import frc.team1126.subsystems.Storage;

public class SpinStorage extends Command {

    private Storage storage;
    private boolean sawNote=false;


    public SpinStorage(Storage storage) {
        addRequirements(RobotContainer.storage);
        this.storage = storage;
    }

    @Override
    public void execute() {
        sawNote = storage.hasNote();

if (sawNote)
{
    storage.setStorageWheels(0);
}

        if (storage.hasNote()) {
            while (storage.hasNote()) {
                storage.setStorageWheels(.3);
            }
            sawNote = true;
            storage.setStorageWheels(0);
            storage.setHasNote();
        }
        else{
            storage.setStorageWheels(5);
        }
    }

    @Override
    public void end(boolean interrupted) {
        storage.setStorageWheels(0.0);
    }

    @Override
    public boolean isFinished() {
        if (sawNote) {
            return true;
        }
        return false;
    }
}
