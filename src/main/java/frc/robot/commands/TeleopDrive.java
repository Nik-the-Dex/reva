// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive.Drive;
import java.util.function.DoubleSupplier;

public class TeleopDrive extends Command {
    private final Drive m_drive;
    private final DoubleSupplier m_translationX;
    private final DoubleSupplier m_translationY;
    private final DoubleSupplier m_rotationX;
    private final boolean m_fieldRelative;

    /**
     * Creates a new TeleopDrive command.
     *
     * @param drive         The drive subsystem.
     * @param translationX  The supplier for X speed (forward/backward meters per second).
     * @param translationY  The supplier for Y speed (left/right meters per second).
     * @param rotationX     The supplier for angular velocity (radians per second).
     * @param fieldRelative Whether the movement is relative to the field or the robot chassis.
     */
    public TeleopDrive(
        Drive drive, 
        DoubleSupplier translationX, 
        DoubleSupplier translationY, 
        DoubleSupplier rotationX,
        boolean fieldRelative
    ) {
        this.m_drive = drive;
        this.m_translationX = translationX;
        this.m_translationY = translationY;
        this.m_rotationX = rotationX;
        this.m_fieldRelative = fieldRelative;
        
        // This stops other drive commands from running simultaneously
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        // Runs once when the command scheduling starts
    }

    @Override
    public void execute() {
        // Grab current physical target values from the joystick lambdas
        double xSpeed = m_translationX.getAsDouble();
        double ySpeed = m_translationY.getAsDouble();
        double rSpeed = m_rotationX.getAsDouble();

        ChassisSpeeds chassisSpeeds;

        // Apply orientation framework conversions
        if (m_fieldRelative) {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, ySpeed, rSpeed, m_drive.getRotation()
            );
        } else {
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rSpeed);
        }

        // Push calculated movement targets down to the drive kinematics mapping loop
        m_drive.runVelocity(chassisSpeeds);
    }

    @Override
    public void end(boolean interrupted) {
        // Safely bring the robot to a complete halt when the command ceases
        m_drive.runVelocity(new ChassisSpeeds());
    }

    @Override
    public boolean isFinished() {
        // Default teleop drive command runs infinitely until driving state ends
        return false;
    }
}

