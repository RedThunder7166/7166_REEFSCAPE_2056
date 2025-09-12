// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Inches;
import static frc.robot.subsystems.arm.ArmConstants.pivotIsAtPositionThreshold;
import static frc.robot.subsystems.elevator.ElevatorConstants.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import frc.robot.Constants;

public class ElevatorIOReal implements ElevatorIO {
    private final TalonFX m_leaderMotor = new TalonFX(leaderMotorId, Constants.CANBUS);
    private final TalonFX m_followerMotor = new TalonFX(followerMotorId, Constants.CANBUS);

    private final StatusSignal<Angle> m_leaderMotorPositionSignal = m_leaderMotor.getPosition();
    private final StatusSignal<Current> m_leaderMotorCurrentSignal = m_leaderMotor.getSupplyCurrent();

    private final StatusSignal<Angle> m_followerMotorPositionSignal = m_followerMotor.getPosition();
    private final StatusSignal<Current> m_followerMotorCurrentSignal = m_followerMotor.getSupplyCurrent();

    private double m_motorTargetPosition = positionInitial;

    private final NeutralOut m_neutralRequest = new NeutralOut();
    private final MotionMagicVoltage m_positionRequest = new MotionMagicVoltage(positionHome);

    public ElevatorIOReal() {
        var leaderConfig = new TalonFXConfiguration();
        leaderConfig.MotorOutput.NeutralMode = neutralMode;
        leaderConfig.MotorOutput.Inverted = leaderInverted;

        leaderConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        leaderConfig.CurrentLimits.SupplyCurrentLimit = currentLimit;
        leaderConfig.Feedback.SensorToMechanismRatio = motorReduction;
        leaderConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        leaderConfig.Slot0.kP = pidP;
        leaderConfig.MotionMagic.MotionMagicAcceleration = acceleration;
        leaderConfig.MotionMagic.MotionMagicCruiseVelocity = cruiseVelocity;
        leaderConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        leaderConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = positionMAX;
        leaderConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        leaderConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = positionMIN;

        var followerConfig = new TalonFXConfiguration();
        followerConfig.MotorOutput.NeutralMode = neutralMode;
        followerConfig.MotorOutput.Inverted = followerInverted;

        followerConfig.CurrentLimits = leaderConfig.CurrentLimits;
        followerConfig.Feedback = leaderConfig.Feedback;
        followerConfig.Slot0 = leaderConfig.Slot0;
        followerConfig.MotionMagic = leaderConfig.MotionMagic;

        tryUntilOk(5, () -> m_leaderMotor.getConfigurator().apply(leaderConfig));
        tryUntilOk(5, () -> m_followerMotor.getConfigurator().apply(followerConfig));

        BaseStatusSignal.setUpdateFrequencyForAll(50d, m_leaderMotorPositionSignal, m_leaderMotorCurrentSignal, m_followerMotorPositionSignal, m_followerMotorCurrentSignal);

        m_leaderMotor.setPosition(m_motorTargetPosition);
        m_followerMotor.setPosition(m_motorTargetPosition);

        m_followerMotor.setControl(new StrictFollower(m_leaderMotor.getDeviceID()));
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.targetMotorPositionRotations = m_motorTargetPosition;

        BaseStatusSignal.refreshAll(m_leaderMotorPositionSignal, m_leaderMotorCurrentSignal, m_followerMotorPositionSignal, m_followerMotorCurrentSignal);

        final double leadMotorPositionRotations = m_leaderMotorPositionSignal.getValueAsDouble();
        inputs.leadMotorPositionRotations = leadMotorPositionRotations;
        inputs.leadMotorPositionInches = mechanismPositionToDistance(leadMotorPositionRotations).in(Inches);
        inputs.leadMotorCurrentAmps = m_leaderMotorCurrentSignal.getValueAsDouble();

        final double followerMotorPositionRotations = m_followerMotorPositionSignal.getValueAsDouble();
        inputs.followerMotorPositionRotations = followerMotorPositionRotations;
        inputs.followerMotorPositionInches = mechanismPositionToDistance(followerMotorPositionRotations).in(Inches);
        inputs.followerMotorCurrentAmps = m_followerMotorCurrentSignal.getValueAsDouble();
    }

    private void setControl(ControlRequest request) {
        m_leaderMotor.setControl(request);
    }

    @Override
    public void periodic() {
        setControl(m_neutralRequest);
    }

    @Override
    public void neutral() {
        setControl(m_neutralRequest);
    }

    @Override
    public void goPosition(double position) {
        m_motorTargetPosition = position;
        setControl(m_positionRequest.withPosition(position));
    }

    @Override
    public boolean isAtPosition(double position) {
        return Math.abs(position - m_leaderMotorPositionSignal.getValueAsDouble()) <= isAtPositionThreshold;
    }

    @Override
    public boolean isAtOrAbovePosition(double position) {
        return (m_leaderMotorPositionSignal.getValueAsDouble() - position) >= pivotIsAtPositionThreshold;
    }
}
