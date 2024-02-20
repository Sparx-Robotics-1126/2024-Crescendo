// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team1126;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
// import frc.team1126.commands.auto.Autos;
import frc.team1126.subsystems.CANdleSubsystem;
import frc.team1126.subsystems.sensors.Limelight;

public class Robot extends TimedRobot {
  private Command autonomousCommand;
  public static RobotContainer robotContainer;
  public static int ledColor;
  public static final CANdleSubsystem m_candleSubsystem = new CANdleSubsystem();


  @Override
  public void robotInit() {
    robotContainer = new RobotContainer();
    // Autos.init();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    // if(Limelight.getInstance().inRange() ) {
    //   m_candleSubsystem.setLEDState(CANdleSubsystem.LEDState.GREEN);
    // } 
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    if ( DriverStation.getAlliance().get() == Alliance.Red){
      ledColor = 0;
      m_candleSubsystem.setLEDState(CANdleSubsystem.LEDState.RED);
    }
    else{
      ledColor = 1;
      m_candleSubsystem.setLEDState(CANdleSubsystem.LEDState.BLU);
    }
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
//    robotContainer.setMotorBrake(true);
    autonomousCommand = robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (autonomousCommand != null) autonomousCommand.cancel();
    // robotContainer.swerve.zeroGyro();
    // RobotContainer.swerve.stopAllModules();
  }

  @Override
  public void teleopPeriodic() {
   
    if(RobotContainer.storage.getHasNote()) {
      
      m_candleSubsystem.setLEDState(CANdleSubsystem.LEDState.YELLOW);
    } else {
      
      if (ledColor == 0) {
        m_candleSubsystem.setLEDState(CANdleSubsystem.LEDState.RED);
      } else if (ledColor == 1) {
        m_candleSubsystem.setLEDState(CANdleSubsystem.LEDState.BLU);
      }
    }
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
