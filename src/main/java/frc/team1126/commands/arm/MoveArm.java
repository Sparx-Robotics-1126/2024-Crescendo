package frc.team1126.commands.arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team1126.RobotContainer;
import frc.team1126.subsystems.Arm;

public class MoveArm extends Command {
    private final double m_power;
    private final Arm m_arm;

    public MoveArm(Arm arm, DoubleSupplier power) {
        addRequirements(RobotContainer.m_arm);
        m_arm = arm;
        m_power = power.getAsDouble();
    }
    
    public MoveArm(Arm arm, Double power) {
        addRequirements(RobotContainer.m_arm);
        m_arm = arm;
        m_power = power;
    }


    @Override
    public void execute(){
        double speed = MathUtil.applyDeadband(m_power, .1);

        if (m_power > 0){
            m_arm.moveArm(speed);
        }
        else if (m_power < 0 && !m_arm.m_homeLimit.get()) {
            m_arm.moveArm(speed);
        } else {
            m_arm.moveArm(0);
        }
    }

    @Override
    public void end(boolean interupted){
        m_arm.moveArm(0);
    }

    @Override
    public boolean isFinished() {
      return false;
    }
}
