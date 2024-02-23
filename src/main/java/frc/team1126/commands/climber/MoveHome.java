package frc.team1126.commands.climber;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team1126.RobotContainer;
import frc.team1126.subsystems.Climber;

public class MoveHome extends Command {
    private final double m_power;
    private final Climber m_climber;

    public MoveHome(Climber climber, double power) {
        addRequirements(RobotContainer.m_climber);
        m_climber = climber;
        m_power = -power;
    }

    @Override
    public void execute() {
        double speedLeft = MathUtil.applyDeadband(m_power, .1);
        double speedRight = MathUtil.applyDeadband(m_power, .1);

        var actualLeftDistance = m_climber.getLeftPosition();

        if ((!m_climber.isLeftHome() || actualLeftDistance <= 0) && speedLeft < 0) {
            m_climber.setLeftPower(0);
            //set position 0
            m_climber.zeroLeft();
        } else {
            m_climber.setLeftPower(speedLeft);
        }

        var actualRightDistance = m_climber.getRightPosition();

        if ((!m_climber.isRightHome() || actualRightDistance <= 0) && speedRight < 0) {
            m_climber.setRightPower(0);
            //set position 0
            m_climber.zeroRight();
        } else {
            m_climber.setRightPower(speedRight);
        }
    }

}
