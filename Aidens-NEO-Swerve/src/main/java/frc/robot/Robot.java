// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private GenericEntry FLdriveS, FLturningS, FLAbsoluteS,
      BLdriveS, BLturningS, BLAbsoluteS,
      BRdriveS, BRturningS, BRAbsoluteS,
      FRdriveS, FRturningS, FRAbsoluteS,
      FLdriveVelocity, FRdriveVelocity, BLdriveVelocity, BRdriveVelocity,
      FLturningVelocity, FRturningVelocity, BLturningVelocity, BRturningVelocity;

  private CANSparkMax FLdriveMotor = new CANSparkMax(DriveConstants.kFrontLeftDriveMotorPort, MotorType.kBrushless);
  private CANSparkMax FLturningMotor = new CANSparkMax(DriveConstants.kFrontLeftTurningMotorPort, MotorType.kBrushless);
  private SwerveEncoder FLabosluteEncoder = new SwerveEncoder(DriveConstants.kFrontLeftDriveAbsoluteEncoderPort);

  private CANSparkMax BLdriveMotor = new CANSparkMax(DriveConstants.kBackLeftDriveMotorPort, MotorType.kBrushless);
  private CANSparkMax BLturningMotor = new CANSparkMax(DriveConstants.kBackLeftTurningMotorPort, MotorType.kBrushless);
  private SwerveEncoder BLabosluteEncoder = new SwerveEncoder(DriveConstants.kBackLeftDriveAbsoluteEncoderPort);

  private CANSparkMax BRdriveMotor = new CANSparkMax(DriveConstants.kBackRightDriveMotorPort, MotorType.kBrushless);
  private CANSparkMax BRturningMotor = new CANSparkMax(DriveConstants.kBackRightTurningMotorPort, MotorType.kBrushless);
  private SwerveEncoder BRabosluteEncoder = new SwerveEncoder(DriveConstants.kBackRightDriveAbsoluteEncoderPort);

  private CANSparkMax FRdriveMotor = new CANSparkMax(DriveConstants.kFrontRightDriveMotorPort, MotorType.kBrushless);
  private CANSparkMax FRturningMotor = new CANSparkMax(DriveConstants.kFrontRightTurningMotorPort, MotorType.kBrushless);
  private SwerveEncoder FRabosluteEncoder = new SwerveEncoder(DriveConstants.kFrontRightDriveAbsoluteEncoderPort);

  @Override
  public void robotInit() {
    FLdriveMotor.setIdleMode(IdleMode.kBrake);
    FRdriveMotor.setIdleMode(IdleMode.kBrake);
    BLdriveMotor.setIdleMode(IdleMode.kBrake);
    BRdriveMotor.setIdleMode(IdleMode.kBrake);

    FLturningMotor.setIdleMode(IdleMode.kBrake);
    FRturningMotor.setIdleMode(IdleMode.kBrake);
    BLturningMotor.setIdleMode(IdleMode.kBrake);
    BRturningMotor.setIdleMode(IdleMode.kBrake);


    ShuffleboardTab programmerBoard = Shuffleboard.getTab("Programmer Board");
    ShuffleboardTab velocityBoard = Shuffleboard.getTab("Velocity Board");

    FLdriveS = programmerBoard.add("FL Drive Count", 0).getEntry();
    FLturningS = programmerBoard.add("FL Turning Count", 0).getEntry();
    FLAbsoluteS = programmerBoard.add("FL Absolute Value", 0).getEntry();

    BLdriveS = programmerBoard.add("BL Drive Count", 0).getEntry();
    BLturningS = programmerBoard.add("BL Turning Count", 0).getEntry();
    BLAbsoluteS = programmerBoard.add("BL Absolute Value", 0).getEntry();

    BRdriveS = programmerBoard.add("BR Drive Count", 0).getEntry();
    BRturningS = programmerBoard.add("BR Turning Count", 0).getEntry();
    BRAbsoluteS = programmerBoard.add("BR Absolute Value", 0).getEntry();

    FRdriveS = programmerBoard.add("FR Drive Count", 0).getEntry();
    FRturningS = programmerBoard.add("FR Turning Count", 0).getEntry();
    FRAbsoluteS = programmerBoard.add("FR Absolute Value", 0).getEntry();

    FLdriveVelocity = velocityBoard.add("FL Drive Vel", 0).getEntry();
    FLturningVelocity = velocityBoard.add("FL Turning Vel", 0).getEntry();

    FRdriveVelocity = velocityBoard.add("FR Drive Vel", 0).getEntry();
    FRturningVelocity = velocityBoard.add("FR Turning Vel", 0).getEntry();
    
    BLdriveVelocity = velocityBoard.add("BL Drive Vel", 0).getEntry();
    BLturningVelocity = velocityBoard.add("BL Turning Vel", 0).getEntry();
    
    BRdriveVelocity = velocityBoard.add("BR Drive Vel", 0).getEntry();
    BRturningVelocity = velocityBoard.add("BR Turning Vel", 0).getEntry();
  }

  @Override
  public void robotPeriodic() {
    FLdriveS.setDouble(FLdriveMotor.getEncoder().getPosition() * ModuleConstants.kDriveEncoderRot2Meter);
    FLturningS.setDouble(FLturningMotor.getEncoder().getPosition() * ModuleConstants.kTurningEncoderRot2Rad);
    FLAbsoluteS.setDouble(FLabosluteEncoder.getPosition());

    BLdriveS.setDouble(BLdriveMotor.getEncoder().getPosition() * ModuleConstants.kDriveEncoderRot2Meter);
    BLturningS.setDouble(BLturningMotor.getEncoder().getPosition() * ModuleConstants.kTurningEncoderRot2Rad);
    BLAbsoluteS.setDouble(BLabosluteEncoder.getPosition());

    BRdriveS.setDouble(BRdriveMotor.getEncoder().getPosition() * ModuleConstants.kDriveEncoderRot2Meter);
    BRturningS.setDouble(BRturningMotor.getEncoder().getPosition() * ModuleConstants.kTurningEncoderRot2Rad);
    BRAbsoluteS.setDouble(BRabosluteEncoder.getPosition());

    FRdriveS.setDouble(FRdriveMotor.getEncoder().getPosition() * ModuleConstants.kDriveEncoderRot2Meter);
    FRturningS.setDouble(FRturningMotor.getEncoder().getPosition() * ModuleConstants.kTurningEncoderRot2Rad);
    FRAbsoluteS.setDouble(FRabosluteEncoder.getPosition());

    FLdriveVelocity.setDouble(FLdriveMotor.getEncoder().getVelocity() * ModuleConstants.kDriveEncoderRPMS2MeterPerSec);
    FLturningVelocity.setDouble(FLturningMotor.getEncoder().getVelocity() * ModuleConstants.kTurningEncoderRPMS2RadPerSec);
    
    FRdriveVelocity.setDouble(FRdriveMotor.getEncoder().getVelocity() * ModuleConstants.kDriveEncoderRPMS2MeterPerSec);
    FRturningVelocity.setDouble(FRturningMotor.getEncoder().getVelocity() * ModuleConstants.kTurningEncoderRPMS2RadPerSec);
    
    BLdriveVelocity.setDouble(BLdriveMotor.getEncoder().getVelocity() * ModuleConstants.kDriveEncoderRPMS2MeterPerSec);
    BLturningVelocity.setDouble(BLturningMotor.getEncoder().getVelocity() * ModuleConstants.kTurningEncoderRPMS2RadPerSec);
    
    BRdriveVelocity.setDouble(BRdriveMotor.getEncoder().getVelocity() * ModuleConstants.kDriveEncoderRPMS2MeterPerSec);
    BRturningVelocity.setDouble(BRturningMotor.getEncoder().getVelocity() * ModuleConstants.kTurningEncoderRPMS2RadPerSec);
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {

    FLdriveMotor.getEncoder().setPosition(0);
    FLturningMotor.getEncoder().setPosition(0);

    BLdriveMotor.getEncoder().setPosition(0);
    BLturningMotor.getEncoder().setPosition(0);

    BRdriveMotor.getEncoder().setPosition(0);
    BRturningMotor.getEncoder().setPosition(0);

    FRdriveMotor.getEncoder().setPosition(0);
    FRturningMotor.getEncoder().setPosition(0);
  }

  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void testInit() {
    FLdriveMotor.getEncoder().setPosition(0);
    FLturningMotor.getEncoder().setPosition(0);

    BLdriveMotor.getEncoder().setPosition(0);
    BLturningMotor.getEncoder().setPosition(0);

    BRdriveMotor.getEncoder().setPosition(0);
    BRturningMotor.getEncoder().setPosition(0);

    FRdriveMotor.getEncoder().setPosition(0);
    FRturningMotor.getEncoder().setPosition(0);
  }

  @Override
  public void testPeriodic() {
    /* 
    FRdriveMotor.set(ControlMode.PercentOutput, 1);
    BRdriveMotor.set(ControlMode.PercentOutput, 1);
    FLdriveMotor.set(ControlMode.PercentOutput, 1);
    BLdriveMotor.set(ControlMode.PercentOutput, 1);
   */
    
    FRturningMotor.set(1);
    BRturningMotor.set(1);
    FLturningMotor.set(1);
    BLturningMotor.set(1);
  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
