// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drive;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
// this is a retooling of the code from 108s 2026 robot for a drive chassis. (half of it is not used and is just here to figure out stuff)
public class DriveConstants {
  public static final double maxSpeedMetersPerSec = 4.6;
  public static final double slowSpeedMetersPerSec = 1.0;
  public static final double odometryFrequency = 100.0; // Hz
  public static final double trackWidth = Units.inchesToMeters(21.5);
  public static final double wheelBase = Units.inchesToMeters(25.5);
  public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
  public static final Translation2d[] moduleTranslations =
      new Translation2d[] {
        new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)
      };

  
  public static final Rotation2d frontLeftZeroRotation = new Rotation2d(-Math.PI / 2);
  public static final Rotation2d frontRightZeroRotation = new Rotation2d(0.0);
  public static final Rotation2d backLeftZeroRotation = new Rotation2d(Math.PI);
  public static final Rotation2d backRightZeroRotation = new Rotation2d(Math.PI / 2);

  public static final int pigeonCanId = 9;

  // Drive motors (Kraken X60 / TalonFX)
  public static final int frontLeftDriveCanId = 1;
  public static final int frontRightDriveCanId = 3;
  public static final int backLeftDriveCanId = 5;
  public static final int backRightDriveCanId = 7;

  // Turn motors (Kraken X44 / TalonFX)
  public static final int frontLeftTurnCanId = 2;
  public static final int frontRightTurnCanId = 4;
  public static final int backLeftTurnCanId = 6;
  public static final int backRightTurnCanId = 8;

  // CANcoder absolute encoder ID
  public static final int frontLeftEncoderCanId = 10;
  public static final int frontRightEncoderCanId = 11;
  public static final int backLeftEncoderCanId = 12;
  public static final int backRightEncoderCanId = 13;

  // Drive motor configuration 
  public static final int driveMotorCurrentLimit = 50;
  public static final double wheelRadiusMeters = Units.inchesToMeters(2.0);
  public static final double driveMotorReduction = 6.75;
  public static final DCMotor driveGearbox = DCMotor.getKrakenX60(1);

  public static final double driveEncoderPositionFactor = 2 * Math.PI / driveMotorReduction;
  public static final double driveEncoderVelocityFactor = (2 * Math.PI) / 60.0 / driveMotorReduction;

  // Drive PID / FeedFordward 
  public static final double driveKp = 0.05;
  public static final double driveKd = 0.0;
  public static final double driveKs = 0.1;
  public static final double driveKv = 0.12;
  public static final double driveSimP = 0.1;
  public static final double driveSimD = 0.0;
  public static final double driveSimKs = 0.0;
  public static final double driveSimKv = 0.0789;

  // Turn motor configuration
  public static final boolean turnInverted = false;
  public static final int turnMotorCurrentLimit = 40;
  public static final double turnMotorReduction = 21.43;
  public static final DCMotor turnGearbox = DCMotor.getKrakenX44(1);

  // CANcoder absolute position in rotations (radians)
  public static final double turnEncoderPositionFactor = 2 * Math.PI;
  public static final double turnEncoderVelocityFactor = (2 * Math.PI) / 60.0;

  // Turn PID 
  public static final double turnKp = 7.0;
  public static final double turnKd = 0.0;
  public static final double turnSimP = 8.0;
  public static final double turnSimD = 0.0;
  public static final double turnPIDMinInput = 0;
  public static final double turnPIDMaxInput = 2 * Math.PI;
 // Gear Ratios
  public static final double R1 = 7.03;
  public static final double R2 = 6.03;
  public static final double R3 = 5.27;
  
}