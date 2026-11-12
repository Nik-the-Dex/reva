// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class GyroIOPigeon2 implements GyroIO {
    private final Pigeon2 pigeon;
    private final StatusSignal<Angle> yaw;
    private final StatusSignal<AngularVelocity> yawVelocity;

    public GyroIOPigeon2() {
        pigeon = new Pigeon2(DriveConstants.pigeonCanId);
        
        // Reset to factory defaults and zero yaw on boot
        pigeon.getConfigurator().apply(new Pigeon2Configuration());
        pigeon.setYaw(0.0);

        yaw = pigeon.getYaw();
        yawVelocity = pigeon.getAngularVelocityZWorld();

        // Match update frequency to DriveConstants
        BaseStatusSignal.setUpdateFrequencyForAll(
            DriveConstants.odometryFrequency, 
            yaw, 
            yawVelocity
        );
        pigeon.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        StatusCode status = BaseStatusSignal.refreshAll(yaw, yawVelocity);
        inputs.connected = status.isOK();
        inputs.yawRad = Units.degreesToRadians(yaw.getValueAsDouble());
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());
    }

    @Override
    public void resetYaw() {
        pigeon.setYaw(0.0);
    }
}
