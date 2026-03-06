// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Subsystems.AutoOperationsController;
import frc.robot.Subsystems.Consts.ShooterConsts;
import frc.robot.Subsystems.Shooter.Shooter;
//import frc.robot.Utils.GamePieceDetector;
import frc.robot.Utils.EverKit.Periodic;
//import frc.robot.Utils.GamePieceCamera.GamePieceType

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;
  public static ArrayList<Periodic> robotPeriodicFuncs = new ArrayList<Periodic>();
  public static ArrayList<Periodic> teleopPeriodicFuncs = new ArrayList<Periodic>();
  public static ArrayList<Periodic> testPeriodicFuncs = new ArrayList<Periodic>();
  public static ArrayList<Periodic> autonomousPeriodicFuncs = new ArrayList<Periodic>();
  public static ArrayList<Periodic> simulationPeriodicFuncs = new ArrayList<Periodic>();
  private RobotContainer m_robotContainer;

  public static Alliance m_alliance;
  

    public Robot() { 
      m_robotContainer = new RobotContainer();
  
    }
  
    @Override
    public void robotInit() {
      m_robotContainer = new RobotContainer();
    
      Logger.recordMetadata("RobotCode-ReBuild-26", "29.01");
      Logger.addDataReceiver(new NT4Publisher());
      Logger.start();
    }


  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    SmartDashboard.putBoolean("setAlliance", SmartDashboard.getBoolean("setAlliance", true));
    if(SmartDashboard.getBoolean("setAlliance", true))
        m_alliance = Alliance.Blue;
    else      m_alliance = Alliance.Red;

    Shooter.getInstance().ConfigureAllianceShootingSetting(m_alliance == Alliance.Blue);
    AutoOperationsController.getInstance().setAlliance(m_alliance == Alliance.Blue);



    SmartDashboard.putNumber("angle",ShooterConsts.ANGLE_MOTOR.getControllerInstance().getAbsoluteEncoder().getPosition());

  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    //m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    //ShooterConsts.ANGLE_MOTOR.set(-0.2);

  }

  @Override
  public void teleopPeriodic() {

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