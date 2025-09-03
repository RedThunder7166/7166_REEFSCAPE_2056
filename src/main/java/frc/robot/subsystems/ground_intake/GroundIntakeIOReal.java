// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ground_intake;

import static frc.robot.subsystems.ground_intake.GroundIntakeConstants.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;

import frc.robot.OurRobotState;
import frc.robot.OurRobotState.ScoreMechanismState;

public class GroundIntakeIOReal implements GroundIntakeIO {
    private final TalonFX m_rollerMotor = new TalonFX(rollerMotorId);
    private final TalonFX m_actuatorMotor = new TalonFX(actuatorMotorId);

    // private final DigitalInput m_beamBreakSensor = new DigitalInput(beamBreakId);

    private final NeutralOut m_neturalRequest = new NeutralOut();

    private final MotionMagicVoltage m_actuatorPositionRequest = new MotionMagicVoltage(actuatorPositionHome);

    private final DutyCycleOut m_rollerDutyCycleRequest = new DutyCycleOut(rollerOutput);

    public GroundIntakeIOReal() {
        var rollerConfig = new TalonFXConfiguration();
        rollerConfig.MotorOutput.NeutralMode = rollerNeutralMode;
        rollerConfig.MotorOutput.Inverted = rollerInverted;

        rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        rollerConfig.CurrentLimits.SupplyCurrentLimit = rollerCurrentLimit;

        var actuatorConfig = new TalonFXConfiguration();
        actuatorConfig.MotorOutput.NeutralMode = actuatorNeutralMode;
        actuatorConfig.MotorOutput.Inverted = actuatorInverted;
        // FIXME: "must be configured so that the sensor reports a position of 0 when the mechanism is horizontal"
        actuatorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        actuatorConfig.Slot0.kP = actuatorPidP;
        actuatorConfig.MotionMagic.MotionMagicAcceleration = actuatorAcceleration;
        actuatorConfig.MotionMagic.MotionMagicCruiseVelocity = actuatorCruiseVelocity;


        tryUntilOk(5, () -> m_rollerMotor.getConfigurator().apply(rollerConfig));
        tryUntilOk(5, () -> m_actuatorMotor.getConfigurator().apply(actuatorConfig));

        m_actuatorMotor.setPosition(actuatorPositionHome);
    }

    @Override
    public void periodic() {
        if (OurRobotState.getScoreMechanismState() == ScoreMechanismState.CORAL_INTAKE && OurRobotState.getIsCoralHolderFirstSensorTripped()) {
            stopRoller();
            retract();
            OurRobotState.setScoreMechanismState(ScoreMechanismState.HOME);
        }
    }

    @Override
    public void updateInputs(GroundIntakeIOInputs inputs) {

    }

    @Override
    public void deploy() {
        m_actuatorMotor.setControl(m_actuatorPositionRequest.withPosition(actuatorPositionDeployed));
    }

    @Override
    public void retract() {
        m_actuatorMotor.setControl(m_actuatorPositionRequest.withPosition(actuatorPositionHome));
    }

    @Override
    public void startRoller() {
        m_rollerMotor.setControl(m_rollerDutyCycleRequest);
    }

    @Override
    public void stopRoller() {
        m_rollerMotor.setControl(m_neturalRequest);
    }
}
