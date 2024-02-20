// package frc.team1126.commands.Shooter;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.team1126.RobotContainer;
// import frc.team1126.subsystems.Shooter;
// import java.util.function.DoubleSupplier;

// import edu.wpi.first.math.MathUtil;

// public class ManualShoot extends Command{
    
//     private final DoubleSupplier power;
//     private Shooter shooter;

//     public ManualShoot(Shooter shooter, DoubleSupplier power) {
//         addRequirements(RobotContainer.shooter);
        
//         this.power = power;
//         this.shooter = shooter;
//     }

//     @Override
//     public void execute(){
//         double speed = MathUtil.applyDeadband(power.getAsDouble(), .1);

//        shooter.setShooterSpeed(speed);
//     }

//     @Override
//     public void end(boolean interupted){
//         shooter.setShooterSpeed(0);
//     }

//     @Override
//     public boolean isFinished() {
//       return false;
//     }
// }
