// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveConstants;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveSubsystem extends SubsystemBase {
  private final WPI_VictorSPX m_leftFrontMotor = new WPI_VictorSPX(DriveConstants.kLeftFrontMotorPort);
  private final WPI_VictorSPX m_rightFrontMotor = new WPI_VictorSPX(DriveConstants.kRightFrontMotorPort);
  private final WPI_VictorSPX m_leftBackMotor = new WPI_VictorSPX(DriveConstants.kLeftBackMotorPort);
  private final WPI_VictorSPX m_rightBackMotor = new WPI_VictorSPX(DriveConstants.kRightBackMotorPort);

  /** Creates a new ExampleSubsystem. */
  // CONSTRUCTOR
  public DriveSubsystem() {
  
    m_leftFrontMotor.setInverted(true);
    m_leftBackMotor.setInverted(true);

  }

  private final MotorControllerGroup leftMotors = new MotorControllerGroup(m_leftFrontMotor,m_leftBackMotor);
  private final MotorControllerGroup rightMotors = new MotorControllerGroup(m_rightFrontMotor,m_rightBackMotor);
  private final DifferentialDrive diffDrive = new DifferentialDrive(leftMotors, rightMotors);

  private Encoder leftEncoder = new Encoder(DriveConstants.kLeftEncoderChannelA, DriveConstants.kLeftEncoderChannelB);
  private Encoder rightEncoder = new Encoder(DriveConstants.kRightEncoderChannelA, DriveConstants.kRightEncoderChannelB);

  public void setMotors(double moveSpeed, double turnSpeed)
  {
    diffDrive.arcadeDrive(moveSpeed, turnSpeed * DriveConstants.returnLimit);
  }

  public double getEncoderFeetAverage()
  {
    return (getLeftEncoderFeet() + getRightEncoderFeet() / 2);
  }
            
  public double getLeftEncoderFeet() {
    double leftEncoderFeet = leftEncoder.get() * DriveConstants.kEncoderTick2Feet;
    return leftEncoderFeet;
  }

  public double getRightEncoderFeet() {
    double rightEncoderFeet = -rightEncoder.get() * DriveConstants.kEncoderTick2Feet;
    return rightEncoderFeet;
  }

  public void resetEncoders()
  {
    leftEncoder.reset();
    rightEncoder.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left Encoder Feet", getLeftEncoderFeet());
    SmartDashboard.putNumber("Right Encoder Feet", getRightEncoderFeet());
    SmartDashboard.putNumber("AVERAGE Encoder Feet", getEncoderFeetAverage());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
