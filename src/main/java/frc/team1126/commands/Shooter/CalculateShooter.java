package frc.team1126.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team1126.RobotContainer;
import frc.team1126.subsystems.Shooter;
import frc.team1126.subsystems.sensors.Limelight;

public class CalculateShooter extends Command {
    
    private Shooter m_shooter;
    private Limelight m_limelight;

    public CalculateShooter(Shooter shooter, Limelight limelight) {
        addRequirements(RobotContainer.m_shooter, RobotContainer.m_limeLight);
        m_shooter = shooter;
        m_limelight = limelight;
    }

    @Override
    public void execute() {
       if(m_limelight.hasHumanTarget()) {
        m_shooter.setShooterPID(m_shooter.calculateShooter());
       } else {
        m_shooter.setShooterPID(m_shooter.calculateShooterOld());
       }
       
    }

    @Override
    public void end(boolean interrupted) {
         m_shooter.setShooterSpeed(0.0);
    }

}
