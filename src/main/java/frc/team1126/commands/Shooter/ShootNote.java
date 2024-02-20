package frc.team1126.commands.Shooter;

import java.util.Timer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team1126.RobotContainer;
import frc.team1126.Constants.ShooterConstants;
import frc.team1126.subsystems.Shooter;
import frc.team1126.subsystems.Storage;

public class ShootNote extends Command{

    private Shooter shooter;
    private Storage storage;
    private long startTime;

    public ShootNote(Shooter shooter, Storage storage) {
        addRequirements(RobotContainer.shooter, RobotContainer.storage);
        this.shooter = shooter;
        this.storage = storage;
    }

    @Override
    public void initialize() {
        startTime = System.currentTimeMillis();
    }

// wait until wheels get up to speed, when they do move storage forward, then resetNote
   @Override
   public void execute() {


    // while(shooter.getShooterSpeed() != ShooterConstants.SHOOTER_SPEED) {
        shooter.setShooterSpeed(1.0);
    // }
    // wont work
    storage.setStorageWheels(1);
    storage.resetNote();
    
   }

   @Override
   public void end(boolean interrupted) {
    shooter.setShooterSpeed(0);
    storage.resetNote();
   }

   
   @Override
   public boolean isFinished() {
       return System.currentTimeMillis() - startTime >= 3000;
   }
}
