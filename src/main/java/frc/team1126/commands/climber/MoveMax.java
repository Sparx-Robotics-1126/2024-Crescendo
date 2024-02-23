package frc.team1126.commands.climber;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team1126.RobotContainer;
import frc.team1126.subsystems.Climber;
import static frc.team1126.Constants.ClimberConstants.*;

public class MoveMax extends Command {
    private final double m_power;
    private final Climber m_climber;

    public MoveMax(Climber climber, double  power) {
        addRequirements(RobotContainer.m_climber);
        m_climber = climber;
        m_power = power;
    }

    @Override
    public void execute() {
        double speedLeft = MathUtil.applyDeadband(m_power, .1);
        double speedRight = MathUtil.applyDeadband(m_power, .1);

        var actualLeftDistance = m_climber.getLeftPosition();

        if (actualLeftDistance > MAX_HEIGHT && speedLeft > 0) {
            m_climber.setLeftPower(0);
        } else {
            m_climber.setLeftPower(speedLeft);
        }

        var actualRightDistance = m_climber.getRightPosition();

        if (actualRightDistance > MAX_HEIGHT && speedRight > 0) {
            m_climber.setRightPower(0);
        } else {
            m_climber.setRightPower(speedRight);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_climber.setLeftPower(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
