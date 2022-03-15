package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class CompressorSubsystem extends SubsystemBase {
    Compressor compressor = new Compressor(26, PneumaticsModuleType.REVPH);
    boolean enabled;
    double current;
    double pressure;

    public CompressorSubsystem() {
        compressor.enableAnalog(100, 115);
        enabled = compressor.enabled();
        current = compressor.getCurrent();
        pressure = compressor.getPressure();
    }

    @Override
    public void periodic() {
        //enabled = compressor.enabled();
        //current = compressor.getCurrent();
        //pressure = compressor.getPressure();
        //SmartDashboard.putBoolean("Compressor Running", enabled);
        //SmartDashboard.putNumber("Compressor Current", current);
        //SmartDashboard.putNumber("Pneumatics Pressure", pressure);
    }
}
