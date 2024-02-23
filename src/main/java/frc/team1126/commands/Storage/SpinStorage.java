package frc.team1126.commands.Storage;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team1126.RobotContainer;
import frc.team1126.subsystems.Storage;

public class SpinStorage extends Command {

    private Storage m_storage;
    private boolean m_sawNote = false;


    public SpinStorage(Storage storage) {
        addRequirements(RobotContainer.m_storage);
        m_storage = storage;
    }

    @Override
    public void execute() {
        m_sawNote = m_storage.hasNote();

if (m_sawNote)
{
    m_storage.setStorageWheels(0);
}

        if (m_storage.hasNote()) {
            while (m_storage.hasNote()) {
                m_storage.setStorageWheels(.2);
            }
            m_sawNote = true;
            m_storage.setStorageWheels(0);
            m_storage.setHasNote();
        }
        else{
            m_storage.setStorageWheels(5);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_storage.setStorageWheels(0.0);
    }

    @Override
    public boolean isFinished() {
        if (m_sawNote) {
            return true;
        }
        return false;
    }
}
