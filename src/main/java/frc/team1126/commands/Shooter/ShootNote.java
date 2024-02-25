package frc.team1126.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team1126.RobotContainer;
import frc.team1126.subsystems.Shooter;
import frc.team1126.subsystems.Storage;

public class ShootNote extends Command{

    private Shooter m_shooter;
    private Storage m_storage;
    private long m_startTime;

    public ShootNote(Shooter shooter, Storage storage) {
        addRequirements(RobotContainer.m_shooter, RobotContainer.m_storage);
        m_shooter = shooter;
        m_storage = storage;
    }

    @Override
    public void initialize() {
        m_startTime = System.currentTimeMillis();
    }

// wait until wheels get up to speed, when they do move storage forward, then resetNote
   @Override
   public void execute() {


    // while(shooter.getShooterSpeed() != ShooterConstants.SHOOTER_SPEED) {
        m_shooter.setShooterSpeed(.6);
    // }
    // wont work
    m_storage.setStorageWheels(1);
    m_storage.resetNote();
    
   }

   @Override
   public void end(boolean interrupted) {
    m_shooter.setShooterSpeed(0);
    m_storage.resetNote();
    m_storage.setStorageWheels(0);
   }

   
   @Override
   public boolean isFinished() {
       return System.currentTimeMillis() - m_startTime >= 3000;
   }
}
