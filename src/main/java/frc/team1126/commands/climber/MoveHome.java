package frc.team1126.commands.climber;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team1126.RobotContainer;
import frc.team1126.subsystems.Climber;

public class MoveHome extends Command {
    private final double power;
    private final Climber climber;

    public MoveHome(Climber climber, double power) {
        addRequirements(RobotContainer.climber);
        this.climber = climber;
        this.power = -power;
    }

    @Override
    public void execute() {
        double speedLeft = MathUtil.applyDeadband(power, .1);
        double speedRight = MathUtil.applyDeadband(power, .1);

        var actualLeftDistance = climber.getLeftPosition();

        if ((!climber.isLeftHome() || actualLeftDistance <= 0) && speedLeft < 0) {
            climber.setLeftPower(0);
        } else {
            climber.setLeftPower(speedLeft);
        }

        var actualRightDistance = climber.getRightPosition();

        if ((!climber.isRightHome() || actualRightDistance <= 0) && speedRight < 0) {
            climber.setRightPower(0);
        } else {
            climber.setRightPower(speedRight);
        }
    }

}
