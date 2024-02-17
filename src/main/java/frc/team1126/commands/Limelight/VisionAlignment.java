// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.team1126.commands.Limelight;

// import java.util.function.DoubleSupplier;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.team1126.Constants.LimelightConstants;
// import frc.team1126.subsystems.SwerveSubsystem;
// import frc.team1126.subsystems.sensors.Limelight;

// public class VisionAlignment extends Command {
//     /** Creates a new VisionAlignment. */
//     private static double x_P = 0.20;
//     private static double x_I = 0.0;
//     private static double x_D = 0.0;
//     private static double z_P = 12.5;
//     private static double z_I = 0.0;
//     private static double z_D = 0.0;
//     public Limelight limelight = new Limelight();
    
//     private static byte visionPipeline =0;// Constants.Limelight.Pipeline_Score;
//     public static void setVisionPipeline(byte pipeline) {
//         visionPipeline = pipeline;
//         // SmartDashboard.putBoolean("VisionAlignment/TargetIsScore", visionPipeline == Constants.Limelight.Pipeline_Score);
//         // SmartDashboard.putBoolean("VisionAlignment/TargetIsCone", visionPipeline == Constants.Limelight.Pipeline_Cone);
//         // SmartDashboard.putBoolean("VisionAlignment/TargetIsCube", visionPipeline == Constants.Limelight.Pipeline_Cube);
//     }
//     public static byte getVisionPipelien(){
//         return visionPipeline;
//     }
    
//     static{
//        // loadConfigs();
//     } 
//     private SwerveSubsystem swerve; 
//     private PIDController xController; 
//     private ProfiledPIDController zController;  
//     private DoubleSupplier forwardSpeed; 

//     public VisionAlignment(DoubleSupplier forwardSpeed, double angle, SwerveSubsystem swerve) {
//         addRequirements(swerve);
//         configPIDs(swerve);
//         xController.setSetpoint(0);
//         zController.setGoal(Units.degreesToRadians(angle));
//         this.forwardSpeed = forwardSpeed; 
//         this.swerve = swerve; 
//         zController.enableContinuousInput(-Math.PI, Math.PI); 
//         setVisionPipeline(visionPipeline);
//     }

//     public void initialize() {
//         xController.reset();
//         zController.reset(swerve.getPose().getRotation().getRadians());

//         // limelight.setPipeline(visionPipeline);
// //        limelight.setLED(visionPipeline == Constants.Limelight.Pipeline_Score ? LED.On : LED.Off);
// //        limelight.setCameraMode(false);
//     }

//     public void execute() {
//         if (limelight.hasTarget()) {
//             double angle = limelight.getAngle();
//             double theta = swerve.getPose().getRotation().getRadians();
//             double yspeed = xController.calculate(-angle);
//             double zSpeed = zController.calculate(theta);
//             SmartDashboard.putNumber("Error", xController.getPositionError());
//             SmartDashboard.putNumber("Speed", yspeed);
//             setPosition(yspeed, zSpeed);
//         }
//         else {
//             setPosition(0,0);
//         }
//     }

//     public boolean isFinished() {
//         return false;
//     }

//     public void end(boolean interrupted) {
//         setPosition(0.0, 0.0); 
//         limelight.setLimelightPipelineIndex(LimelightConstants.APRILTAG_PIPELINE);
//  //       limelight.setLED(LED.Off);
//  //       limelight.setCameraMode(true);
//     }

//     private void setPosition(double speed, double zSpeed) {
//         swerve.drive(new Translation2d(forwardSpeed.getAsDouble(),speed), 0.0, true, true);
//         // swerve.driveSpeed(forwardSpeed.getAsDouble(), speed, 0.0, true);   
//     }

//     private void configPIDs(SwerveSubsystem swerve) {
//         xController = new PIDController(x_P, x_I, x_D);
//         zController = new ProfiledPIDController(z_P, z_I, z_D,
//                 new TrapezoidProfile.Constraints(swerve.getMaxAngularSpeed(), swerve.getMaxAngularAcceleration()));
//         xController.setTolerance(1);
//     }

// }