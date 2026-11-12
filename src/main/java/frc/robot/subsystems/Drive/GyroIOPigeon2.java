// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drive;

import com.ctre.phoenix6.hardware.Pigeon2;

public class GyroIOPigeon2 implements GyroIO {
    private final Pigeon2 pigeon;

    public GyroIOPigeon2(int canID) {
        pigeon = new Pigeon2(canID);
  
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.yawRad = Math.toRadians(pigeon.getYaw().getValueAsDouble());
        inputs.yawVelocityRadPerSec = Math.toRadians(pigeon.getAngularVelocityZWorld().getValueAsDouble());
    }
}
