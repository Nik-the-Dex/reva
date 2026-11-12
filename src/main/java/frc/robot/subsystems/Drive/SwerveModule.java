// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.Logger;

public class SwerveModule {
    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    private final String name;
    
    private final PIDController turnFeedback = new PIDController(0.0, 0.0, 0.0);
    private final double wheelRadiusMeters = (2.25 * 0.0254) / 2.0;

    public SwerveModule(String name, ModuleIO io) {
        this.name = name;
        this.io = io;
        turnFeedback.enableContinuousInput(-Math.PI, Math.PI);
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Drive/" + name, inputs);
    }

    public void setTargetState(SwerveModuleState state) {
        @SuppressWarnings("deprecation")
        SwerveModuleState optimized = SwerveModuleState.optimize(state, getAngle());
        
        //linear speed into rad/sec
        double speedRadPerSec = optimized.speedMetersPerSecond / wheelRadiusMeters;
        io.setDriveVoltage(speedRadPerSec * 0.1); // Feedfoward loop

        double turnVoltage = turnFeedback.calculate(getAngle().getRadians(), optimized.angle.getRadians());
        io.setTurnVoltage(turnVoltage);
    }

    public Rotation2d getAngle() {
        return new Rotation2d(inputs.turnAbsolutePositionRad);
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(inputs.drivePositionRad * wheelRadiusMeters, getAngle());
    }
}
