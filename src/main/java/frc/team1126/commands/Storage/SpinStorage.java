package frc.team1126.commands.Storage;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team1126.RobotContainer;
import frc.team1126.subsystems.Storage;

public class SpinStorage extends Command {

    private Storage m_storage;
    private double m_power;
    private boolean m_sawNote = false;
private boolean holdingNote = false;

    public 
    SpinStorage(Storage storage, double power) {
        addRequirements(RobotContainer.m_storage);
        m_storage = storage;
        m_sawNote =false;
        holdingNote = false;
        m_power = power;
    }

    @Override
    public void execute() {

        m_storage.setStorageWheels(5);
        // if (!m_sawNote){
        //     m_storage.setStorageWheels(5);
        // }
        if (m_storage.hasNote()){
            // m_sawNote = true;
            holdingNote = true;
            m_storage.setStorageWheels(m_power);
        }
        else if (holdingNote && !m_storage.hasNote()) {
            m_storage.setStorageWheels(0);
            m_storage.setHasNote();
            m_sawNote = true;
        }

        // m_sawNote = m_storage.hasNote();



// if (m_sawNote)
// {
//     m_storage.setStorageWheels(0);
// }

//         if (m_storage.hasNote()) {
//             if (m_storage.hasNote()) {
//                 m_storage.setStorageWheels(.2);
//             }
//             m_sawNote = true;
//             m_storage.setStorageWheels(0);
//             m_storage.setHasNote();
//         }
//         else{
//             m_storage.setStorageWheels(5);
//         }
    }

    @Override
    public void end(boolean interrupted) {
        m_storage.setStorageWheels(0.0);
    }

    @Override
    public boolean isFinished() {
        if (m_sawNote && holdingNote) {
            m_sawNote = false;
            holdingNote = false;
            return true;
        }
        return false;
    }
}
