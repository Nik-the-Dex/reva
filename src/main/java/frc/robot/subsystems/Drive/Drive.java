// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.Drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    private final SwerveModule[] modules;
    private final SwerveDriveKinematics kinematics;

    public Drive(GyroIO gyroIO, ModuleIO flIO, ModuleIO frIO, ModuleIO blIO, ModuleIO brIO, SwerveDriveKinematics kinematics) {
        this.gyroIO = gyroIO;
        this.kinematics = kinematics;

        modules = new SwerveModule[] {
            new SwerveModule("FrontLeft", flIO),
            new SwerveModule("FrontRight", frIO),
            new SwerveModule("BackLeft", blIO),
            new SwerveModule("BackRight", brIO)
        };
    }

    @Override
    public void periodic() {
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Drive/Gyro", gyroInputs);

        for (SwerveModule module : modules) {
            module.periodic();
        }
    }

    public void runVelocity(ChassisSpeeds speeds) {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, 4.5); // Max 4.5 m/s reference

        for (int i = 0; i < 4; i++) {
            modules[i].setTargetState(states[i]);
        }
    }

    public Rotation2d getRotation() {
        return new Rotation2d(gyroInputs.yawRad);
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            positions[i] = modules[i].getPosition();
        }
        return positions;
    }
}

