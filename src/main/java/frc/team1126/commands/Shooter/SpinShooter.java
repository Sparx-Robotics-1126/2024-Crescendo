package frc.team1126.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team1126.RobotContainer;
import frc.team1126.subsystems.Shooter;

public class SpinShooter extends Command {
    

    private Shooter m_shooter;
    private double m_power;

    public SpinShooter(Shooter shooter, double power) {

        addRequirements(RobotContainer.m_shooter);
        m_shooter = shooter;
        m_power = power;

    }

    @Override
    public void execute(){
       // m_shooter.setShooterSpeed(m_power);
       m_shooter.setShooterPID(m_power);
    }

       @Override
   public void end(boolean interrupted) {
        m_shooter.setShooterSpeed(0.0);
   }

   
   @Override
   public boolean isFinished() {
     return false;
   }
}
