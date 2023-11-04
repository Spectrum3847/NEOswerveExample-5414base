// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.text.DecimalFormat;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.SwerveConstants;

public class Drivetrain extends SubsystemBase {
  private SwerveModule leftFront = new SwerveModule(
    SwerveConstants.LEFT_FRONT_DRIVE_ID, 
    SwerveConstants.LEFT_FRONT_TURN_ID, 
    false, 
    true, 
    SwerveConstants.LEFT_FRONT_CANCODER_ID, 
    SwerveConstants.LEFT_FRONT_OFFSET, 
    false);

  private SwerveModule rightFront = new SwerveModule(
    SwerveConstants.RIGHT_FRONT_DRIVE_ID, 
    SwerveConstants.RIGHT_FRONT_TURN_ID, 
    false, 
    true, 
    SwerveConstants.RIGHT_FRONT_CANCODER_ID, 
    SwerveConstants.RIGHT_FRONT_OFFSET, 
    false);

  private SwerveModule leftBack = new SwerveModule(
    SwerveConstants.LEFT_BACK_DRIVE_ID, 
    SwerveConstants.LEFT_BACK_TURN_ID, 
    false, 
    true, 
    SwerveConstants.LEFT_BACK_CANCODER_ID, 
    SwerveConstants.LEFT_BACK_OFFSET, 
    false);

  private SwerveModule rightBack = new SwerveModule(
    SwerveConstants.RIGHT_BACK_DRIVE_ID, 
    SwerveConstants.RIGHT_BACK_TURN_ID, 
    false, 
    true, 
    SwerveConstants.RIGHT_BACK_CANCODER_ID, 
    SwerveConstants.RIGHT_BACK_OFFSET, 
    false);

  private SlewRateLimiter frontLimiter = new SlewRateLimiter(SwerveConstants.TELE_DRIVE_MAX_ACCELERATION);
  private SlewRateLimiter sideLimiter = new SlewRateLimiter(SwerveConstants.TELE_DRIVE_MAX_ACCELERATION);
  private SlewRateLimiter turnLimiter = new SlewRateLimiter(SwerveConstants.TELE_DRIVE_MAX_ANGULAR_ACCELERATION);

  private Pigeon2 gyro = new Pigeon2(SwerveConstants.PIGEON_ID);
  private double rates[] = new double[3];

  private enum DriveMode{
    kNormal, kSubs, kArmGrid, kShooterGrid, kX
  }

  DriveMode mode = DriveMode.kNormal;

  private static final Drivetrain drivetrain = new Drivetrain();

  public static Drivetrain getInstance(){
    return drivetrain;
  }

  /** Creates a new SwerveDrivetrain. */
  public Drivetrain() {
    new Thread(() -> {
      try{
        Thread.sleep(1000);
        zeroHeading();
      }
      catch(Exception e){}
    }).start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    RobotContainer.poseEstimator.updateOdometry(getHeadingRotation2d(), getModulePositions());

    gyro.getRawGyro(rates);
    SmartDashboard.putNumber("Robot Angle", getHeading());
    SmartDashboard.putNumber("Robot Pitch", getPitch());
    SmartDashboard.putNumber("Robot Roll", getRoll());
    SmartDashboard.putString("Pose", getPose().toString());
    SmartDashboard.putString("Angular Speed", new DecimalFormat("#.00").format((rates[2] / 180)) + "pi rad/s");

    Logger.getInstance().recordOutput("Drivetrain/Robot Angle", getHeadingRotation2d().getRadians());
    Logger.getInstance().recordOutput("Drivetrain/Pose", getPose());
    Logger.getInstance().recordOutput("Drivetrain/Angular Speed", rates[2] / 180);
    Logger.getInstance().recordOutput("Drivetrain/Module States", getModuleStates());
  }

  public void swerveDrive(double frontSpeed, double sideSpeed, double turnSpeed, 
    boolean fieldOriented, Translation2d centerOfRotation, boolean deadband){ //Drive with rotational speed control w/ joystick
    if(deadband){
      frontSpeed = Math.abs(frontSpeed) > 0.1 ? frontSpeed : 0;
      sideSpeed = Math.abs(sideSpeed) > 0.1 ? sideSpeed : 0;
      turnSpeed = Math.abs(turnSpeed) > 0.1 ? turnSpeed : 0;
    }

    frontSpeed = RobotContainer.driverController.getRightTriggerAxis() > 0.5 ? frontSpeed * 0.20 : frontSpeed;
    sideSpeed = RobotContainer.driverController.getRightTriggerAxis() > 0.5 ? sideSpeed * 0.20 : sideSpeed;
    turnSpeed = RobotContainer.driverController.getRightTriggerAxis() > 0.5 ? turnSpeed * 0.20 : turnSpeed;

    frontSpeed = frontLimiter.calculate(frontSpeed) * SwerveConstants.TELE_DRIVE_MAX_SPEED;
    sideSpeed = sideLimiter.calculate(sideSpeed) * SwerveConstants.TELE_DRIVE_MAX_SPEED;
    turnSpeed = turnLimiter.calculate(turnSpeed) * SwerveConstants.TELE_DRIVE_MAX_ANGULAR_SPEED;

    ChassisSpeeds chassisSpeeds;
    if(fieldOriented){
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(frontSpeed, sideSpeed, turnSpeed, getHeadingRotation2d());
    }
    else{
      chassisSpeeds = new ChassisSpeeds(frontSpeed, sideSpeed, turnSpeed);
    }

    SwerveModuleState[] moduleStates = SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds, centerOfRotation);

    setModuleStates(moduleStates);
  }

  public void swerveDrive(double frontSpeed, double sideSpeed, double turnX, double turnY, 
    boolean fieldOriented, Translation2d centerOfRotation, boolean deadbandX, boolean deadbandY, boolean deadbandTurn){ //Drive with rotational heading control w/ joystick
    if(deadbandX){
      frontSpeed = Math.abs(frontSpeed) > 0.1 ? frontSpeed : 0;
    }
    if(deadbandY){
      sideSpeed = Math.abs(sideSpeed) > 0.1 ? sideSpeed : 0;
    }
    if(deadbandTurn){
      turnX = Math.abs(turnX) > 0.1 ? turnX : 0;
      turnY = Math.abs(turnY) > 0.1 ? turnY : 0;
    }

    double turnSpeed;
    if(turnX == 0 && turnY == 0){
      turnSpeed = 0;
    }
    else{
      double error = getJoystickAngle(turnX, turnY) - getHeading();
    
      if(error > 180) {
        error -= 360;
      }
      else if(error < -180){
        error += 360;
      }
    
      if(Math.abs(error) > 1){
        turnSpeed = Math.signum(error) * SwerveConstants.kS_PERCENT + SwerveConstants.kP_PERCENT * error;
      }
      else{
        turnSpeed = 0;
      }
    }

    frontSpeed = RobotContainer.driverController.getLeftTriggerAxis() > 0.5 ? frontSpeed * 0.20 : frontSpeed;
    sideSpeed = RobotContainer.driverController.getLeftTriggerAxis() > 0.5 ? sideSpeed * 0.20 : sideSpeed;
    turnSpeed = RobotContainer.driverController.getLeftTriggerAxis() > 0.5 ? turnSpeed * 0.20 : turnSpeed;

    frontSpeed = frontLimiter.calculate(frontSpeed) * SwerveConstants.TELE_DRIVE_MAX_SPEED;
    sideSpeed = sideLimiter.calculate(sideSpeed) * SwerveConstants.TELE_DRIVE_MAX_SPEED;
    turnSpeed = turnLimiter.calculate(turnSpeed) * SwerveConstants.TELE_DRIVE_MAX_ANGULAR_SPEED;

    ChassisSpeeds chassisSpeeds;
    if(fieldOriented){
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(frontSpeed, sideSpeed, turnSpeed, getHeadingRotation2d());
    }
    else{
      chassisSpeeds = new ChassisSpeeds(frontSpeed, sideSpeed, turnSpeed);
    }

    SwerveModuleState[] moduleStates = SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds, centerOfRotation);

    setModuleStates(moduleStates);
  }

  public void turnToHeading(double heading, Translation2d centerOfRotation){
    double turnSpeed;
    double error = heading - getHeading();

    if(error > 180) {
      error -= 360;
    }
    else if(error < -180){
      error += 360;
    }
    
    if(Math.abs(error) > 1){
      turnSpeed = Math.signum(error) * SwerveConstants.kS_PERCENT + SwerveConstants.kP_PERCENT * error;
    }
    else{
      turnSpeed = 0;
    }

    turnSpeed = turnLimiter.calculate(turnSpeed) * SwerveConstants.TELE_DRIVE_MAX_ANGULAR_SPEED;

    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, turnSpeed, getHeadingRotation2d());

    SwerveModuleState[] moduleStates = SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds, centerOfRotation);

    setModuleStates(moduleStates);
  }
  
  public Pose2d getPose(){
    return RobotContainer.poseEstimator.getEstimatedPose();
  }

  public void resetOdometry(Pose2d pose){
    RobotContainer.poseEstimator.resetPose(pose);
  }

  public void setAllIdleMode(boolean brake){
    if(brake){
      leftFront.setBrake(true);
      rightFront.setBrake(true);
      leftBack.setBrake(true);
      rightBack.setBrake(true);
    }
    else{
      leftFront.setBrake(false);
      rightFront.setBrake(false);
      leftBack.setBrake(false);
      rightBack.setBrake(false);
    }
  }

  public void resetAllEncoders(){
    leftFront.resetEncoders();
    rightFront.resetEncoders();
    leftBack.resetEncoders();
    rightBack.resetEncoders();
  }

  public void zeroHeading(){
    gyro.setYaw(0);
  }

  public void setHeading(double heading){
    gyro.setYaw(heading);
  }

  public double getHeading(){
    return Math.IEEEremainder(gyro.getYaw(), 360); //clamp heading between -180 and 180
  }

  public Rotation2d getHeadingRotation2d(){
    return Rotation2d.fromDegrees(getHeading());
  }

  public double getRoll(){
    return gyro.getRoll() + 0.92;
  }

  public double getPitch(){
    return gyro.getPitch() + 0.13;
  }

  public void formX(){
    leftFront.setRawState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    rightFront.setRawState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    leftBack.setRawState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    rightBack.setRawState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  public void stopModules(){
    leftFront.stop();
    leftBack.stop();
    rightFront.stop();
    rightBack.stop();
  }

  public void setModuleStates(SwerveModuleState[] moduleStates){
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, SwerveConstants.DRIVETRAIN_MAX_SPEED);
    leftFront.setDesiredState(moduleStates[0]);
    rightFront.setDesiredState(moduleStates[1]);
    leftBack.setDesiredState(moduleStates[2]);
    rightBack.setDesiredState(moduleStates[3]);
  }

  public SwerveModuleState[] getModuleStates(){
    SwerveModuleState[] states = new SwerveModuleState[4];
    states[0] = leftFront.getState();
    states[1] = rightFront.getState();
    states[2] = leftBack.getState();
    states[3] = rightBack.getState();
    return states;
  } 

  public SwerveModulePosition[] getModulePositions(){
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    positions[0] = leftFront.getPosition();
    positions[1] = rightFront.getPosition();
    positions[2] = leftBack.getPosition();
    positions[3] = rightBack.getPosition();
    return positions;
  } 

  public double getJoystickAngle(double turnX, double turnY){
    double targetAngle;

    if(turnX != 0){
      targetAngle = Math.toDegrees(Math.atan2(-turnX, turnY));   
    }
    else{
      targetAngle = Math.toDegrees(Math.atan2(turnX, turnY));
    }
    
    return targetAngle;
  }

  public DriveMode getDriveMode(){
    return mode;
  }

  public DriveMode getNormalMode(){
    return DriveMode.kNormal;
  }

  public DriveMode getXMode(){
    return DriveMode.kX;
  }

  public void setNormalMode(){
    mode = DriveMode.kNormal;
  }

  public void setXMode(){
    mode = DriveMode.kX;
  }
}
