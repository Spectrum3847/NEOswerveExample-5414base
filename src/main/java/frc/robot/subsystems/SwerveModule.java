// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.PearadoxSparkMax;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule extends SubsystemBase {
  private PearadoxSparkMax driveMotor;
  private PearadoxSparkMax turnMotor;

  private RelativeEncoder driveEncoder;
  private RelativeEncoder turnEncoder;

  private PIDController turnPIDController;
  private CANCoder absoluteEncoder;

  private boolean absoluteEncoderReversed;
  private double absoluteEncoderOffset;

  private Rotation2d lastAngle;

  /** Creates a new SwerveModule. */
  public SwerveModule(int driveMotorId, int turnMotorId, boolean driveMotorReversed, boolean turnMotorReversed,
    int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {
      this.absoluteEncoderOffset = absoluteEncoderOffset;
      this.absoluteEncoderReversed = absoluteEncoderReversed;

      absoluteEncoder = new CANCoder(absoluteEncoderId);

      driveMotor = new PearadoxSparkMax(driveMotorId, MotorType.kBrushless, IdleMode.kCoast, 45, driveMotorReversed);
      turnMotor = new PearadoxSparkMax(turnMotorId, MotorType.kBrushless, IdleMode.kCoast, 25, turnMotorReversed);

      driveEncoder = driveMotor.getEncoder();
      turnEncoder = turnMotor.getEncoder();

      turnPIDController = new PIDController(SwerveConstants.KP_TURNING, 0, 0);
      turnPIDController.enableContinuousInput(-Math.PI, Math.PI);

      resetEncoders();
      lastAngle = getState().angle;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setBrake(boolean brake){
    if(brake){
      driveMotor.setIdleMode(IdleMode.kBrake);
      turnMotor.setIdleMode(IdleMode.kCoast);
    }
    else{
      driveMotor.setIdleMode(IdleMode.kCoast);
      turnMotor.setIdleMode(IdleMode.kCoast);
    }
  }
  
  public double getDriveMotorPosition(){
    return driveEncoder.getPosition() * SwerveConstants.DRIVE_MOTOR_PCONVERSION;
  }

  public double getDriveMotorVelocity(){
    return driveEncoder.getVelocity() * SwerveConstants.DRIVE_MOTOR_VCONVERSION;
  }

  public double getTurnMotorPosition(){
    return turnEncoder.getPosition() * SwerveConstants.TURN_MOTOR_PCONVERSION;
  }

  public double getTurnMotorVelocity(){
    return turnEncoder.getVelocity() * SwerveConstants.TURN_MOTOR_VCONVERSION;
  }

  public double getAbsoluteEncoderAngle(){
    double angle = absoluteEncoder.getAbsolutePosition();
    angle -= absoluteEncoderOffset;
    angle *= (Math.PI / 180);
    return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
  }

  public void resetEncoders(){
    driveEncoder.setPosition(0);
    turnEncoder.setPosition(getAbsoluteEncoderAngle() / SwerveConstants.TURN_MOTOR_PCONVERSION);
  }

  public SwerveModuleState getState(){
    return new SwerveModuleState(getDriveMotorVelocity(), new Rotation2d(getTurnMotorPosition()));
  }

  public SwerveModulePosition getPosition(){
    return new SwerveModulePosition(getDriveMotorPosition(), new Rotation2d(getTurnMotorPosition()));
  }

  public void setDesiredState(SwerveModuleState desiredState){
    /* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
    desiredState = SwerveModuleState.optimize(desiredState, getState().angle); 
    setAngle(desiredState);
    setSpeed(desiredState);
    SmartDashboard.putString("Swerve [" + driveMotor.getDeviceId() + "] State", getState().toString());
    Logger.getInstance().recordOutput("Drivetrain/Module " + driveMotor.getDeviceId() + " State", getState());
  }

  public void setRawState(SwerveModuleState desiredState){
    /* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
    desiredState = SwerveModuleState.optimize(desiredState, getState().angle); 
    setRawAngle(desiredState);
    setSpeed(desiredState);
    SmartDashboard.putString("Swerve [" + driveMotor.getDeviceId() + "] State", getState().toString());
    Logger.getInstance().recordOutput("Drivetrain/Module " + driveMotor.getDeviceId() + " State", getState());
  }

  public void setSpeed(SwerveModuleState desiredState){
    driveMotor.set(desiredState.speedMetersPerSecond / SwerveConstants.DRIVETRAIN_MAX_SPEED);

    // driveController.setReference(
    //   desiredState.speedMetersPerSecond,
    //   ControlType.kVelocity,
    //   0,
    //   feedforward.calculate(desiredState.speedMetersPerSecond));
  }

  public void setAngle(SwerveModuleState desiredState){
    Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (SwerveConstants.DRIVETRAIN_MAX_SPEED * 0.01)) ? lastAngle : desiredState.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.
    
    turnMotor.set(turnPIDController.calculate(getTurnMotorPosition(), desiredState.angle.getRadians()));
    lastAngle = angle;
  }

  public void setRawAngle(SwerveModuleState desiredState){
    Rotation2d angle = desiredState.angle;

    turnMotor.set(turnPIDController.calculate(getTurnMotorPosition(), desiredState.angle.getRadians()));
    lastAngle = angle;
  }
  
  public void stop(){
    driveMotor.set(0);
    turnMotor.set(0);
  }
}
