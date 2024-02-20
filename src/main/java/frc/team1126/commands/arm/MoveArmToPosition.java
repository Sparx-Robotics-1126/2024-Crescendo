package frc.team1126.commands.arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team1126.RobotContainer;
import frc.team1126.subsystems.Arm;

public class MoveArmToPosition extends Command {
    private final double targetAngle;
    private final Arm arm;

    public MoveArmToPosition(Arm arm, double targetAngle) { // -3
        addRequirements(RobotContainer.arm);
        this.arm = arm;
        this.targetAngle = targetAngle;
    }

    @Override
    public void execute() {
        double currentPitch = arm.getPitch();
        double target = targetAngle;

        if (currentPitch < target) {
            arm.moveArm(-.5); // positive power to move up
        } else if (currentPitch > target) {
            arm.moveArm(-0.1); // negative power to move down
        }
    }

    @Override
    public void end(boolean interupted) {
        arm.moveArm(0);
    }

    @Override
    public boolean isFinished() {

        if (arm.getPitch() >= targetAngle) {
            return true;
        }
        return false;
    }
}