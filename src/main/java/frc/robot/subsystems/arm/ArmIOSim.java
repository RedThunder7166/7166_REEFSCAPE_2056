// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.subsystems.arm.ArmConstants.*;

import frc.robot.util.BeamBreakSensor;

public class ArmIOSim implements ArmIO {
    private double m_pivotTargetPosition = pivotPositionInitial;
    private BeamBreakSensor m_gripperSensor;

    @Override
    public ArmIOSim withGripperSensor(BeamBreakSensor sensor) {
        m_gripperSensor = sensor;
        return this;
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.gripperSensorTripped = !m_gripperSensor.get();

        // :(
        inputs.pivotMotorPositionRotations = m_pivotTargetPosition;
        inputs.pivotMotorPositionDegrees = mechanismPositionToPivotAngle(m_pivotTargetPosition).in(Degrees);
        inputs.pivotTargetMotorPositionRotations = m_pivotTargetPosition;
        inputs.pivotTargetMotorPositionDegrees = mechanismPositionToPivotAngle(m_pivotTargetPosition).in(Degrees);
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
