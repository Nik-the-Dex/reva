// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.TeleopDrive;
import frc.robot.subsystems.Drive.Drive;
import frc.robot.subsystems.Drive.GyroIO;
import frc.robot.subsystems.Drive.GyroIOPigeon2;
import frc.robot.subsystems.Drive.ModuleIO;
import frc.robot.subsystems.Drive.ModuleIOTalonFX;

public class RobotContainer {
    
    private final CommandXboxController m_driverController = new CommandXboxController(0);

    private final double trackWidth = 0.6; // e.g., 24 inches Frame size
    private final double wheelBase = 0.6;

    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
        new Translation2d(wheelBase / 2.0, trackWidth / 2.0),  // Front Left
        new Translation2d(wheelBase / 2.0, -trackWidth / 2.0), // Front Right
        new Translation2d(-wheelBase / 2.0, trackWidth / 2.0), // Back Left
        new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0) // Back Right
    );

    
    private final Drive m_drive = new Drive(
        new GyroIOPigeon2(0) {}, 
        new ModuleIOTalonFX(1, 2, 9),  // Front Left Module IDs
        new ModuleIOTalonFX(3, 4, 10), // Front Right Module IDs
        new ModuleIOTalonFX(5, 6, 11), // Back Left Module IDs
        new ModuleIOTalonFX(7, 8, 12), // Back Right Module IDs
        m_kinematics
    );

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // MK5n gear ratio stage (speed is guess)
        final double maxLinearSpeed = 4.5;       
        final double maxAngularSpeed = Math.PI * 2; 

        m_drive.setDefaultCommand(
            new TeleopDrive(
                m_drive,
                () -> -modifyAxis(m_driverController.getLeftY()) * maxLinearSpeed,   // X 
                () -> -modifyAxis(m_driverController.getLeftX()) * maxLinearSpeed,   // Y 
                () -> -modifyAxis(m_driverController.getRightX()) * maxAngularSpeed, // Rotational
                true 
            )
        );

    }

    private double modifyAxis(double value) {
        double deadband = 0.08;
        if (Math.abs(value) < deadband) {
            return 0.0;
        }
        return value;
    }

    public Command getAutonomousCommand() {
        // Return your autonomous routines here
        return null;
    }
}
