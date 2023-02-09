package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.Telemetry;
import frc.robot.Constants.POP;

public class PinchersofPower extends SubsystemBase  {
  private final Compressor comp;
  private final DoubleSolenoid pusher;
  private final CANSparkMax spinner;
  private final CANSparkMax spinner2;
  private boolean m_cone;

  public PinchersofPower() {
    comp = new Compressor(1, PneumaticsModuleType.CTREPCM);
    pusher = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 0);
    spinner = new CANSparkMax(25, MotorType.kBrushless);
    spinner2 = new CANSparkMax(25, MotorType.kBrushless);
    spinner2.follow(spinner);
    m_cone = false;
  }

  public void forward() {
    pusher.set(Value.kForward);
  }

  public void reverse() {
    pusher.set(Value.kReverse);
  }

  public void off() {
    pusher.set(Value.kOff);
  }

  public void spinin() {
    spinner.set(POP.SPEED);
  }

  public void spinout() {
    spinner.set(-POP.SPEED);
  }

  public void spinoff() {
    spinner.set(0);
  }

  public void enable() {
    comp.enableDigital();
  }

  public void disable() {
    comp.disable();
  }

  public void intake() {
    if((pusher.get() == Value.kForward)) {
      reverse();
    }
    spinin();
    if((pusher.get() != Value.kForward) && (m_cone == true)) {
      forward();
    }
  }

  public void outtake() {
    if(m_cone != true) {
      spinout();
    }
    if((m_cone == true) && (pusher.get() == Value.kForward)) {
      pusher.set(Value.kReverse);
    }
  }

  public void notake() {
    spinoff();
    off();
  }

  public void setMode(String mode) {
    if(mode == "cone") {
      m_cone = true;
    }
    if(mode == "cube") {
      m_cone = false;
    }
  }

  public Command Intake(PinchersofPower Claw) {
    return new InstantCommand(() -> intake(), Claw);
  }

  public Command Outtake(PinchersofPower Claw) {
    return new InstantCommand(() -> outtake(), Claw);
  }

  public Command Notake(PinchersofPower Claw) {
    return new InstantCommand(() -> notake(), Claw);
  }

  @Override
  public void periodic() {
    Telemetry.setValue("POP/Spinner/speed", spinner.get());
    Telemetry.setValue("POP/Spinner/temp", spinner.getMotorTemperature());
    Telemetry.setValue("POP/Spinner/voltage", spinner.getAppliedOutput());
    Telemetry.setValue("POP/Spinner/statorcurrent", spinner.getOutputCurrent());
    Telemetry.setValue("POP/SpinnerFollower/speed2", spinner2.get());
    Telemetry.setValue("POP/SpinnerFollower/temp2", spinner2.getMotorTemperature());
    Telemetry.setValue("POP/SpinnerFollower/voltage2", spinner2.getAppliedOutput());
    Telemetry.setValue("POP/SpinnerFollower/statorcurrent2", spinner2.getOutputCurrent());
    Telemetry.setValue("POP/pneumatics/value", pusher.get());
  }
}