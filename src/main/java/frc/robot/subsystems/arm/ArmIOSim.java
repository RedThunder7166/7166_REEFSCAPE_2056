// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import static frc.robot.subsystems.arm.ArmConstants.*;

public class ArmIOSim implements ArmIO {
    private double m_pivotTargetPosition = pivotPositionInitial;

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        // :(
        inputs.pivotMotorPositionRotations = m_pivotTargetPosition;
        inputs.pivotTargetMotorPositionRotations = m_pivotTargetPosition;
    }

    @Override
    public void pivotGoPosition(double position) {
        m_pivotTargetPosition = position;
    }

    @Override
    public boolean pivotIsAtPosition(double position) {
        return m_pivotTargetPosition == position;
    }

    @Override
    public boolean pivotIsAtOrPastPosition(double position) {
        return m_pivotTargetPosition >= position;
    }
}
