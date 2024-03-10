package frc.team1126.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team1126.subsystems.Shooter;

public class CalculateShooter extends Command {
    
    private Shooter m_shooter;

    public CalculateShooter(Shooter shooter) {
        m_shooter = shooter;
    }

    @Override
    public void execute() {
        m_shooter.setShooterPID(m_shooter.calculateShooter());
    }
}
