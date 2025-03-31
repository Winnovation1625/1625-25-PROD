package frc.robot.subsystem.leds;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.VirtualSubsystem;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.Setter;

public class Leds extends VirtualSubsystem {

  private final int HARDWARE_LEDS = 8;
  private final int LEDS_PER_STRIP = 126 - HARDWARE_LEDS;
  private static Leds instance;

  private final CANdle candle;
  private LoggedTunableNumber animationSpeed = new LoggedTunableNumber("LED/AnimationSpeed", 0.25);
  private DoubleSupplier animationBrightness = (() -> DriverStation.isEnabled() ? 0.6 : 0.9);
  @Setter @Getter private boolean algaeInBot = false;
  @Setter @Getter private boolean gamepieceInBot = false;
  @Setter @Getter private boolean endGameWarning = false;
  @Setter @Getter private boolean whenLinedUp = false;
  @Setter @Getter private boolean lowBattery = false;
  @Setter @Getter private Boolean[] aprilTagCams;
  @Setter @Getter private Boolean autoLiningUp = false;
  private Boolean[] lastAprilTagCams = new Boolean[] {true, true, true, true};

  public static Leds getInstance() {
    if (instance == null) {
      instance = new Leds();
    }
    return instance;
  }

  public Leds() {
    candle = new CANdle(0, "canivore");
    // changeAnimation(AnimationTypes.SetAll);
    CANdleConfiguration configAll = new CANdleConfiguration();
    configAll.statusLedOffWhenActive = true;
    configAll.disableWhenLOS = false;
    configAll.stripType = LEDStripType.GRB;
    configAll.brightnessScalar = 1;
    configAll.vBatOutputMode = VBatOutputMode.Off;
    configAll.v5Enabled = true;
    candle.configAllSettings(configAll, 100);
    aprilTagCams = new Boolean[] {false, false, false, false};
  }

  @Override
  public synchronized void periodic() {
    for (int i = 0; i < aprilTagCams.length; i++) {
      if (lastAprilTagCams[i] != aprilTagCams[i]) {
        if (aprilTagCams[i].booleanValue()) {
          if (i == 0) {
            candle.setLEDs(0, 255, 0, 0, 0, 1);
            candle.setLEDs(0, 255, 0, 0, 7, 1);
          } else if (i == 1) {
            candle.setLEDs(0, 255, 0, 0, 1, 1);
            candle.setLEDs(0, 255, 0, 0, 6, 1);
          } else if (i == 2) {
            candle.setLEDs(0, 255, 0, 0, 2, 1);
            candle.setLEDs(0, 255, 0, 0, 5, 1);
          } else {
            candle.setLEDs(0, 255, 0, 0, 3, 2);
          }
        } else {
          if (i == 0) {
            candle.setLEDs(255, 0, 0, 0, 0, 1);
            candle.setLEDs(255, 0, 0, 0, 7, 1);
          } else if (i == 1) {
            candle.setLEDs(255, 0, 0, 0, 1, 1);
            candle.setLEDs(255, 0, 0, 0, 6, 1);
          } else if (i == 2) {
            candle.setLEDs(255, 0, 0, 0, 2, 1);
            candle.setLEDs(255, 0, 0, 0, 5, 1);
          } else {
            candle.setLEDs(255, 0, 0, 0, 3, 2);
          }
          // cams are disconnected
        }
        lastAprilTagCams[i] = aprilTagCams[i];
      }
    }
    if (DriverStation.isEStopped() && DriverStation.isTeleop()) {
      // solid red
      candle.setLEDs(255, 0, 0);
    } else if (DriverStation.isAutonomous() && DriverStation.isEStopped()) {
      if (DriverStation.isEStopped()) {
        // solid yellow
        candle.setLEDs(255, 255, 0);
      }
    } else if (lowBattery) {
      // low battery, red fading
      candle.animate(
          new SingleFadeAnimation(
              255, 0, 0, 0, animationSpeed.get(), LEDS_PER_STRIP, HARDWARE_LEDS),
          0);

    } else if (endGameWarning) {
      // flash red
      candle.animate(
          new RainbowAnimation(
              animationBrightness.getAsDouble(),
              animationSpeed.get(),
              LEDS_PER_STRIP,
              false,
              HARDWARE_LEDS),
          0);
    } else if (whenLinedUp) {
      // Solid Green
      candle.setLEDs(0, 255, 0, 0, HARDWARE_LEDS, LEDS_PER_STRIP);

    } else if (autoLiningUp) { // this is a hidden message from seth. hey cutie.
      candle.animate(
          new FireAnimation(
              animationBrightness.getAsDouble(),
              animationSpeed.get(),
              LEDS_PER_STRIP,
              1.0,
              1.0,
              false,
              HARDWARE_LEDS));
    } else if (gamepieceInBot || algaeInBot) {
      // purple
      candle.setLEDs(255, 0, 255, 0, HARDWARE_LEDS, LEDS_PER_STRIP);
    } else {
      int redValue = AllianceFlipUtil.shouldFlip() ? 255 : 0;
      int blueValue = AllianceFlipUtil.shouldFlip() ? 0 : 255;
      // when no pattern run color flow of alliance color
      // color flow alliance color
      candle.animate(
          new ColorFlowAnimation(
              redValue,
              0,
              blueValue,
              0,
              animationSpeed.get(),
              LEDS_PER_STRIP,
              Direction.Backward,
              HARDWARE_LEDS),
          0);
    }
  }
}
