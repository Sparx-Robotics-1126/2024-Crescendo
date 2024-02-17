package frc.team1126.commands.rotoation;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team1126.RobotContainer;
import frc.team1126.subsystems.Rotation;

public class MoveArm extends Command {
    private final DoubleSupplier power;
    private final Rotation rotation;

    public MoveArm(Rotation rotation, DoubleSupplier power) {
        addRequirements(RobotContainer.rotation);
        this.rotation = rotation;
        this.power = power;
    }

    @Override
    public void execute(){
        double speed = MathUtil.applyDeadband(power.getAsDouble(), .1);

        rotation.moveArm(speed);
    }

    @Override
    public void end(boolean interupted){
        rotation.moveArm(0);
    }

    @Override
    public boolean isFinished() {
      return false;
    }
}
