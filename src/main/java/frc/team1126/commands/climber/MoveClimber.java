package frc.team1126.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team1126.RobotContainer;
import frc.team1126.subsystems.Climber;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import static  frc.team1126.Constants.ClimberConstants.*;

public class MoveClimber extends Command {

  private final DoubleSupplier m_power;
  // private final DoubleSupplier rightPower;
  private final Climber m_climber;

  public MoveClimber(Climber climber, DoubleSupplier power) {
    addRequirements(RobotContainer.m_climber);
    m_climber = climber;
    m_power = power;
  }

  @Override
  public void execute() {
    double speed = MathUtil.applyDeadband(m_power.getAsDouble(), .1);

    // System.out.println("in here " + climber.isLeftHome());
    var actualLeftDistance = m_climber.getLeftPosition();
    if (actualLeftDistance > MAX_HEIGHT && speed > 0) {
      m_climber.setLeftPower(0);
    } else if (!m_climber.isLeftHome() && speed < 0) {
      m_climber.setLeftPower(0);
    } else {
      m_climber.setLeftPower(speed);
    }

    var actualRightDistance = m_climber.getRightPosition();
    if (actualRightDistance >  MAX_HEIGHT && speed > 0) {
      m_climber.setRightPower(0);
    } else if (!m_climber.isRightHome() && speed < 0) {
      m_climber.setRightPower(0);
    } else {
      m_climber.setRightPower(speed);
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
