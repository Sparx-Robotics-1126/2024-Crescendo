package frc.team1126.commands.climber;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team1126.RobotContainer;
import frc.team1126.subsystems.Climber;
import static frc.team1126.Constants.ClimberConstants.*;

public class MoveMax extends Command {
    private final double power;
    private final Climber climber;

    public MoveMax(Climber climber, double  power) {
        addRequirements(RobotContainer.climber);
        this.climber = climber;
        this.power = power;
    }

    @Override
    public void execute() {
        double speedLeft = MathUtil.applyDeadband(power, .1);
        double speedRight = MathUtil.applyDeadband(power, .1);

        var actualLeftDistance = climber.getLeftPosition();

        if (actualLeftDistance > MAX_HEIGHT && speedLeft > 0) {
            climber.setLeftPower(0);
        } else {
            climber.setLeftPower(speedLeft);
        }

        var actualRightDistance = climber.getRightPosition();

        if (actualRightDistance > MAX_HEIGHT && speedRight > 0) {
            climber.setRightPower(0);
        } else {
            climber.setRightPower(speedRight);
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
