// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drive;

import java.util.ArrayDeque;
import java.util.Arrays;
import java.util.Queue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class ModuleIOTalonFX implements ModuleIO {
    private final TalonFX driveMotor;
    private final TalonFX turnMotor; 
    private final CANcoder encoder;

   
    private final Queue<Double> timestampQueue = new ArrayDeque<>();
    private final Queue<Double> drivePositionQueue = new ArrayDeque<>();
    private final Queue<Double> turnPositionQueue = new ArrayDeque<>();

    public ModuleIOTalonFX(int driveID, int turnID, int encoderID) {
        driveMotor = new TalonFX(driveID);
        turnMotor = new TalonFX(turnID);
        encoder = new CANcoder(encoderID);

       
        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        driveMotor.getConfigurator().apply(driveConfig);

        TalonFXConfiguration turnConfig = new TalonFXConfiguration();
        turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        turnConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        turnMotor.getConfigurator().apply(turnConfig);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        
        double driveRotationsToRad = (2.0 * Math.PI) / 6.03; 
        double turnRotationsToRad = (2.0 * Math.PI) / (287.0 / 11.0); 
        
        inputs.drivePositionRad = driveMotor.getPosition().getValueAsDouble() * driveRotationsToRad;
        inputs.driveVelocityRadPerSec = driveMotor.getVelocity().getValueAsDouble() * driveRotationsToRad;

        inputs.turnAbsolutePositionRad = encoder.getAbsolutePosition().getValueAsDouble() * 2.0 * Math.PI;
        inputs.driveAppliedVolts = driveMotor.getMotorVoltage().getValueAsDouble();

        
        inputs.turnVelocityRadPerSec = turnMotor.getVelocity().getValueAsDouble() * turnRotationsToRad; 
        inputs.turnAppliedVolts = turnMotor.getMotorVoltage().getValueAsDouble();

        // Odometry samples streams
        inputs.odometryTimestamps = timestampQueue.stream().mapToDouble(Double::doubleValue).toArray();
        inputs.odometryDrivePositionsRad = drivePositionQueue.stream()
            .mapToDouble(v -> Units.rotationsToRadians(v / frc.robot.subsystems.Drive.DriveConstants.driveMotorReduction))
            .toArray();
        inputs.odometryTurnPositions = turnPositionQueue.stream()
            .map(Rotation2d::fromRotations)
            .toArray(Rotation2d[]::new);

       
        int n = Math.min(
            inputs.odometryTimestamps.length,
            Math.min(inputs.odometryDrivePositionsRad.length, inputs.odometryTurnPositions.length)
        );
        
        if (n < inputs.odometryTimestamps.length)
            inputs.odometryTimestamps = Arrays.copyOf(inputs.odometryTimestamps, n);
        if (n < inputs.odometryDrivePositionsRad.length)
            inputs.odometryDrivePositionsRad = Arrays.copyOf(inputs.odometryDrivePositionsRad, n);
        if (n < inputs.odometryTurnPositions.length)
            inputs.odometryTurnPositions = Arrays.copyOf(inputs.odometryTurnPositions, n);

     
        timestampQueue.clear();
        drivePositionQueue.clear();
        turnPositionQueue.clear();
    }

    @Override
    public void setDriveVoltage(double volts) {
        driveMotor.setVoltage(volts);
    }

    @Override
    public void setTurnVoltage(double volts) {
        turnMotor.setVoltage(volts);
    }
}
