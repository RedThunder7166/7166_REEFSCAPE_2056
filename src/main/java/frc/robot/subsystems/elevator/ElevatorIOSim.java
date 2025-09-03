// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

public class ElevatorIOSim implements ElevatorIO {
    private double m_targetPosition = positionGrab;

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {

    }

    @Override
    public void goPosition(double position) {
        m_targetPosition = position;
    }
    @Override
    public boolean isAtPosition(double position) {
        return m_targetPosition == position;
    }
}
