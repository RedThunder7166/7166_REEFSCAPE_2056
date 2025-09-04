// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.straightenator;

import static frc.robot.subsystems.straightenator.StraightenatorConstants.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.OurRobotState;
import frc.robot.OurRobotState.ScoreMechanismState;

public class StraightenatorIOReal implements StraightenatorIO {
    private final TalonFX m_leftMotor = new TalonFX(leftMotorId);
    private final TalonFX m_rightMotor = new TalonFX(rightMotorId);

    private final StatusSignal<AngularVelocity> m_leftMotorVelocitySignal = m_leftMotor.getVelocity();
    private final StatusSignal<Current> m_leftMotorCurrentSignal = m_leftMotor.getSupplyCurrent();

    private final StatusSignal<AngularVelocity> m_rightMotorVelocitySignal = m_rightMotor.getVelocity();
    private final StatusSignal<Current> m_rightMotorCurrentSignal = m_rightMotor.getSupplyCurrent();

    private final DigitalInput m_firstSensor = new DigitalInput(firstSensorId);
    private final DigitalInput m_secondSensor = new DigitalInput(secondSensorId);

    private final NeutralOut m_neutralRequest = new NeutralOut();

    private final DutyCycleOut m_dutyCycleRequest = new DutyCycleOut(intakeOutput);

    public StraightenatorIOReal() {
        var leftConfig = new TalonFXConfiguration();
        leftConfig.MotorOutput.NeutralMode = neutralMode;
        leftConfig.MotorOutput.Inverted = leftInverted;

        leftConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        leftConfig.CurrentLimits.SupplyCurrentLimit = currentLimit;

        var rightConfig = new TalonFXConfiguration();
        rightConfig.MotorOutput.NeutralMode = neutralMode;
        rightConfig.MotorOutput.Inverted = rightInverted;

        rightConfig.CurrentLimits = leftConfig.CurrentLimits;


        tryUntilOk(5, () -> m_leftMotor.getConfigurator().apply(leftConfig));
        tryUntilOk(5, () -> m_rightMotor.getConfigurator().apply(rightConfig));

        BaseStatusSignal.setUpdateFrequencyForAll(50d, m_leftMotorVelocitySignal, m_leftMotorCurrentSignal, m_rightMotorVelocitySignal, m_rightMotorCurrentSignal);
    }

    @Override
    public void periodic() {
        boolean firstSensorTripped = !m_firstSensor.get();
        boolean secondSensorTripped = !m_secondSensor.get();

        if (OurRobotState.getIsIdle())
            off();
        else {
            if (firstSensorTripped && OurRobotState.getScoreMechanismState() == ScoreMechanismState.CORAL_INTAKE)
                off();

            if (secondSensorTripped)
                off();
        }

        OurRobotState.setIsCoralHolderFirstSensorTripped(firstSensorTripped);
        OurRobotState.setIsCoralHolderSecondSensorTripped(secondSensorTripped);
    }

    @Override
    public void updateInputs(StraightenatorIOInputs inputs) {
        BaseStatusSignal.refreshAll(m_leftMotorVelocitySignal, m_leftMotorCurrentSignal, m_rightMotorVelocitySignal, m_rightMotorCurrentSignal);

        inputs.leftMotorVelocityRPS = m_leftMotorVelocitySignal.getValueAsDouble();
        inputs.leftMotorCurrentAmps = m_leftMotorCurrentSignal.getValueAsDouble();

        inputs.rightMotorVelocityRPS = m_rightMotorVelocitySignal.getValueAsDouble();
        inputs.rightMotorCurrentAmps = m_rightMotorCurrentSignal.getValueAsDouble();
    }

    @Override
    public void on() {
        m_leftMotor.setControl(m_dutyCycleRequest.withOutput(intakeOutput));
        m_rightMotor.setControl(m_dutyCycleRequest.withOutput(intakeOutput));
    }
    @Override
    public void off() {
        m_leftMotor.setControl(m_neutralRequest);
        m_rightMotor.setControl(m_neutralRequest);
    }
}
