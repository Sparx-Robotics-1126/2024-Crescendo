// package frc.team1126.commands.Limelight;

// import com.ctre.phoenix.sensors.PigeonIMU;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.geometry.Rotation3d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.lib.swervelib.imu.SwerveIMU;
// import frc.team1126.subsystems.SwerveSubsystem;
// import frc.team1126.subsystems.sensors.Limelight;

// public class DriveToDistance extends Command {

//     private final SwerveSubsystem swerve;
//     private final Limelight limelight;
//     private final PIDController distanceController;
//     private final double targetDistance;
//     private double initialYaw;

//     public DriveToDistance(SwerveSubsystem swerve, double targetDistance) {
//         this.swerve = swerve;
//         this.limelight = new Limelight();
//         this.targetDistance = targetDistance;
//         this.distanceController = new PIDController(0.15, 0.0, 0.0);
//         this.distanceController.setSetpoint(targetDistance);
//         addRequirements(swerve);
//     }

//     @Override
//     public void initialize() {
//         // Record the initial yaw of the robot
//         initialYaw = Math.toRadians(swerve.getYaw().getDegrees());
//     }

//     @Override
//     public void execute() {
//         if (limelight.hasTarget()) {
//             double currentDistance = limelight.getDistance();
//             double speed = distanceController.calculate(currentDistance);

//             // Calculate the x and y components of the speed based on the initial yaw
//             double xSpeed = Math.cos(initialYaw) * speed;
//             double ySpeed = Math.sin(initialYaw) * speed;

//             // Get the current yaw of the robot from the SwerveSubsystem
//             double currentYaw = swerve.getYaw().getDegrees();

//             // Drive the swerve based on the x and y components of the speed and the current heading
//             swerve.drive(new Translation2d(xSpeed, ySpeed), initialYaw, true, true);
//         }
//         else {
//             // Stop the robot if the Limelight does not have a target
//             swerve.drive(new Translation2d(0, 0), 0.0, true, true);
//         }
//     }

//     @Override
//     public boolean isFinished() {
//         return distanceController.atSetpoint();
//     }
// }