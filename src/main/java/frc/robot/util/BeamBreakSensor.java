package frc.robot.util;

import java.util.Optional;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

@FunctionalInterface
interface OptionalBooleanSupplier {
    Optional<Boolean> getAsOptionalBoolean();
}

public class BeamBreakSensor {
    private DigitalInput m_realInput;
    private OptionalBooleanSupplier m_fakeSupplier;

    public BeamBreakSensor(int realChannel, OptionalBooleanSupplier fakeSupplier) {
        if (realChannel == -1)
            m_realInput = null;
        else
            m_realInput = new DigitalInput(realChannel);
        m_fakeSupplier = fakeSupplier;
    }

    public void setSupplier(OptionalBooleanSupplier fakeSupplier) {
        if (m_fakeSupplier != null)
            throw new RuntimeException("setSupplier cannot be called if a supplier already exists!");
        m_fakeSupplier = fakeSupplier;
    }

    private boolean m_commandExists = false;
    private boolean m_commandValue = true;
    public Command toggleCommand() {
        if (m_commandExists)
            throw new RuntimeException("toggleCommand can only be called once!");
        m_commandExists = true;

        setSupplier(() -> Optional.of(m_commandValue));

        // return Commands.runEnd(() -> {
        //     m_commandValue = false;
        // }, () -> m_commandValue = true);
        return Commands.runOnce(() -> m_commandValue = !m_commandValue);
    }

    public boolean get() {
        if (m_fakeSupplier == null) {
            if (m_realInput == null)
                return true;
            return m_realInput.get();
        } else if (m_realInput == null)
            return m_fakeSupplier.getAsOptionalBoolean().orElse(true);

        return m_fakeSupplier.getAsOptionalBoolean().orElseGet(m_realInput::get);
    }
}
