package frc.team1126.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team1126.RobotContainer;
import frc.team1126.subsystems.Climber;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import static  frc.team1126.Constants.ClimberConstants.*;

public class MoveClimber extends Command {

  private final DoubleSupplier power;
  private final Climber climber;

  public MoveClimber(Climber climber, DoubleSupplier power) {
    addRequirements(RobotContainer.climber);
    this.climber = climber;
    this.power = power;
    // this.rightPower = rightPower;
  }

  @Override
  public void execute() {
    double speed = MathUtil.applyDeadband(power.getAsDouble(), .1);
    // double speedRight = MathUtil.applyDeadband(rightPower.getAsDouble(), .1);

    // System.out.println("in here " + climber.isLeftHome());
    var actualLeftDistance = climber.getLeftPosition();
    if (actualLeftDistance > MAX_HEIGHT && speed > 0) {
      climber.setLeftPower(0);
    } else if (!climber.isLeftHome() && speed < 0) {
      climber.setLeftPower(0);
    } else {
      climber.setLeftPower(speed);
    }

    var actualRightDistance = climber.getRightPosition();
    if (actualRightDistance >  MAX_HEIGHT && speed > 0) {
      climber.setRightPower(0);
    } else if (!climber.isRightHome() && speed < 0) {
      climber.setRightPower(0);
    } else {
      climber.setRightPower(speed);
    }
  }

  @Override
  public void end(boolean interrupted) {
    climber.setLeftPower(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
