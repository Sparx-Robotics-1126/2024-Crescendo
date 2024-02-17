// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team1126;

import java.io.File;
import java.util.HashMap;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.team1126.Constants.SwerveConstants;
// import frc.team1126.commands.climber.MoveClimber;
// import frc.team1126.commands.climber.MoveHome;
// import frc.team1126.commands.climber.MoveMax;
// import frc.team1126.commands.rotoation.MoveArm;
import frc.team1126.subsystems.CANdleSubsystem;
import frc.team1126.subsystems.Climber;
import frc.team1126.subsystems.Rotation;
import frc.team1126.subsystems.SwerveSubsystem;
//import frc.team1126.subsystems.SwerveSubsystem;
import frc.team1126.subsystems.sensors.Limelight;

public class RobotContainer {

  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  // public static final CANdleSubsystem m_candleSubsystem = new  CANdleSubsystem();

  private static HashMap<String, Command> pathMap = new HashMap<>();

  private final SendableChooser<Command> _chooser = new SendableChooser<>();

  CommandXboxController driver = new CommandXboxController(Constants.GeneralConstants.DRIVER_CONTROLLER_ID);
  CommandXboxController operator = new CommandXboxController(Constants.GeneralConstants.OPERATOR_CONTROLLER_ID);

  /* Operator Buttons */
  // private final JoystickButton cubeMode = new JoystickButton(operator.getHID(),
  // XboxController.Button.kStart.value);
  // private final JoystickButton coneMode = new JoystickButton(operator.getHID(),
  // XboxController.Button.kBack.value);

  public final static SwerveSubsystem swerve = new SwerveSubsystem(new  File(Filesystem.getDeployDirectory(), "swerve"));

  public final Limelight m_limeLight = new Limelight();

  // public final static Climber climber = new Climber();

  // public final static Rotation rotation = new Rotation();

  public RobotContainer() {
    configureDriverBindings();
    configureOperatoreBindings();

    Command driveFieldOrientedAnglularVelocity = swerve.driveCommand(
      () -> MathUtil.clamp(MathUtil.applyDeadband(-driver.getLeftY(), .1), -1,
          1),
      () -> MathUtil.clamp(MathUtil.applyDeadband(-driver.getLeftX(),.1), -1,
          1),
      () -> -driver.getRightX());

      swerve.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    // swerve.setDefaultCommand(new DriveFieldRelative(swerve, 
    //                                                 () -> driver.getRawAxis(translationAxis)*-1,
    //                                                 () -> driver.getRawAxis(strafeAxis) *-1,
    //                                                 () -> driver.getRawAxis(rotationAxis)*-1)); 

    // configureChooser();
    // climber.setDefaultCommand(new MoveClimber(climber, () -> -operator.getRawAxis(XboxController.Axis.kLeftY.value)));
    
    // rotation.setDefaultCommand(new MoveArm(rotation, () -> -operator.getRawAxis(XboxController.Axis.kLeftY.value)));
    
    // climber.setDefaultCommand(climber.moveClimber(
    // MathUtil.applyDeadband(operator.getRawAxis(XboxController.Axis.kLeftY.value),
    // .1),
    // MathUtil.applyDeadband(operator.getRawAxis(XboxController.Axis.kLeftX.value),
    // .1)));
    // climber.setDefaultCommand(climber.moveLeftClimber(
    // MathUtil.applyDeadband(operator.getRawAxis(XboxController.Axis.kLeftY.value),
    // .1)));
  }

  private void configureDriverBindings()
  {
  driver.leftTrigger().onTrue(new InstantCommand(() -> swerve.zeroGyro()));



  // driver.a().whileTrue(new VisionAlignment(this::getXSpeed, 0, swerve));
  // driver.x().whileTrue(new LLAlignCommand(true));
  // driver.b().whileTrue(new DriveToDistance(swerve,60));
  // driver.povUp().onTrue(new InstantCommand(() ->
  // swerve.setHeadingAngle(Math.round(swerve.getYaw().getRadians() /
  // (2.0*Math.PI)) * Math.PI * 2)));
  // driver.povDown().onTrue(new InstantCommand(() ->
  // swerve.setHeadingAngle(Math.round(swerve.getYaw().getRadians() /
  // (2.0*Math.PI)) * Math.PI * 2 - Math.PI)));
  // driver.povLeft().onTrue(new InstantCommand(() ->
  // swerve.setHeadingAngle(Math.round(swerve.getYaw().getRadians() /
  // (2.0*Math.PI)) * Math.PI * 2 - Math.PI/2)));
  // driver.povRight().onTrue(new InstantCommand(() ->
  // swerve.setHeadingAngle(Math.round(swerve.getYaw().getRadians() /
  // (2.0*Math.PI)) * Math.PI * 2 + Math.PI/2)));
  // driver.leftBumper().onTrue(new InstantCommand(() ->
  // swerve.setTranslationalScalar(Constants.SwerveConstants.SWERVE_SLOW_TRANSLATION)));
  // driver.leftBumper().onFalse(new InstantCommand(() ->
  // swerve.setTranslationalScalar(Constants.SwerveConstants.SWERVE_NORMAL_TRANSLATION)));
  }

  private void configureOperatoreBindings() {
    // operator.start().onTrue(new InstantCommand(()
    // ->m_candleSubsystem.setLEDState(CANdleSubsystem.LEDState.CONE)));
    // operator.back().onTrue(new InstantCommand(() ->
    // m_candleSubsystem.setLEDState(CANdleSubsystem.LEDState.CUBE)));
    // operator.x().onTrue(new InstantCommand(() ->
    // operator.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 1.0)));


    // operator.leftBumper().whileTrue(new MoveMax(climber, .05));
    // operator.rightBumper().whileTrue(new MoveHome(climber, .05));


    // operator.leftStick().whileTrue(
    // climber.moveLeftClimber(operator.getLeftY()));

  }

  double getXSpeed() {
    int pov = driver.getHID().getPOV();
    double finalX;

    if (pov == 0)
      finalX = -0.05;
    else if (pov == 180)
      finalX = 0.05;
    else if (Math.abs(driver.getLeftY()) <= 0.1)
      finalX = 0.0;
    else
      finalX = driver.getLeftY() * 0.75 * (1.0 + driver.getLeftTriggerAxis());

    return finalX;
  }

  public double getYSpeed() {
    int pov = driver.getHID().getPOV();

    double finalY;
    if (pov == 270 || pov == 315 || pov == 225)
      finalY = -0.05;
    else if (pov == 90 || pov == 45 || pov == 135)
      finalY = 0.05;
    else if (Math.abs(driver.getLeftX()) <= 0.1)
      finalY = 0.0;
    else
      finalY = driver.getLeftX() * 0.75 * (1.0 + driver.getLeftTriggerAxis());

    // if (SwerveType.isStandard())
    // finalY = -finalY;
    return finalY;
  }

  // public void configureChooser() {

  // //_chooser.setDefaultOption("Do Nothing", new InstantCommand());
  // _chooser.setDefaultOption("2M AUTO", new PathPlannerAuto("2M Auto"));
  // _chooser.addOption("5M Auto", new PathPlannerAuto("5M Auto"));

  // SmartDashboard.putData("AUTO CHOICES ", _chooser);

  // }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // swerve.zeroGyro();
    //
    // // m_driveSubsystem.setHeading(180);
    // Timer.delay(0.05);
    // // the command to be run in autonomous
    // SmartDashboard.putData("AUTO CHOICES ", _chooser);
    // return _chooser.getSelected();
    return _chooser.getSelected();
    // return swerve.getAutonomousCommand(_chooser.getSelected().getName(), true);

  }

  public void EndGameRumble() {

    if (DriverStation.getMatchTime() < SwerveConstants.EndGameSeconds
        && DriverStation.getMatchTime() > SwerveConstants.StopRumbleSeconds) {

      driver.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.5);
      operator.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.5);

    } else {
      driver.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0);
      operator.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0);

    }
  }

}
