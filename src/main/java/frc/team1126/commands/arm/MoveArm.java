package frc.team1126.commands.arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team1126.RobotContainer;
import frc.team1126.subsystems.Arm;

public class MoveArm extends Command {
    private final DoubleSupplier power;
    private final Arm rotation;

    public MoveArm(Arm rotation, DoubleSupplier power) {
        addRequirements(RobotContainer.arm);
        this.rotation = rotation;
        this.power = power;
    }

    @Override
    public void execute(){
        while(!rotation.getHomeLimit()) {
            double speed = MathUtil.applyDeadband(power.getAsDouble(), .1);

            rotation.moveArm(speed);
        }
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
