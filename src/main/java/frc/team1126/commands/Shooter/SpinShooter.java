package frc.team1126.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team1126.RobotContainer;
import frc.team1126.subsystems.Shooter;

public class SpinShooter extends Command {
    

    private Shooter shooter;

    public SpinShooter(Shooter shooter) {

        addRequirements(RobotContainer.shooter);
        this.shooter = shooter;

    }

    @Override
    public void execute(){
        shooter.setShooterSpeed(1.0);
    }

       @Override
   public void end(boolean interrupted) {
       shooter.setShooterSpeed(0.0);
   }

   
   @Override
   public boolean isFinished() {
     return false;
   }
}
