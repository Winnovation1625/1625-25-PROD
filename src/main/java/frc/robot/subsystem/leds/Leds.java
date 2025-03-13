package frc.robot.subsystem.leds;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.VirtualSubsystem;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.Setter;

public class Leds extends VirtualSubsystem {
  private final int LEDS_PER_STRIP = 124;
  private final int NUM_LED_STRIPS = 1;
  private final int HARDWARE_LEDS = 8;
  private static Leds instance;

  private final CANdle candle;
  private LoggedTunableNumber animationSpeed = new LoggedTunableNumber("LED/AnimationSpeed", 0.25);
  private DoubleSupplier animationBrightness = (() -> DriverStation.isEnabled() ? 0.6 : 0.9);
  @Setter @Getter private boolean algaeInBot = false;
  @Setter @Getter private boolean coralInBot = false;
  @Setter @Getter private boolean endGameWarning = false;
  @Setter @Getter private boolean whenLinedUp = false;
  @Setter @Getter private boolean lowBattery = false;
  @Setter @Getter private Boolean[] aprilTagCams;
  private Boolean[] lastAprilTagCams = new Boolean[] {true, true, true, true};
  private boolean lastObjectDetectionCam = true;

  private boolean isRed = false;

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
    candle.animate(
        new RainbowAnimation(
            animationBrightness.getAsDouble(), animationSpeed.get(), LEDS_PER_STRIP, true, 0));
    // for (int i = 0; i < aprilTagCams.length; i++) {
    //   if (lastAprilTagCams[i] != aprilTagCams[i]) {
    //     if (aprilTagCams[i].booleanValue()) {
    //       if (i == 0) {
    //         candle.setLEDs(0, 255, 0, 0, 0, 1);
    //       } else if (i == 1) {
    //         candle.setLEDs(0, 255, 0, 0, 3, 1);
    //       } else if (i == 2) {
    //         candle.setLEDs(0, 255, 0, 0, 7, 1);
    //       } else {
    //         candle.setLEDs(0, 255, 0, 0, 4, 1);
    //       }
    //     } else {
    //       if (i == 0) {
    //         candle.setLEDs(255, 0, 0, 0, 0, 1);
    //       } else if (i == 1) {
    //         candle.setLEDs(255, 0, 0, 0, 3, 1);
    //       } else if (i == 2) {
    //         candle.setLEDs(255, 0, 0, 0, 7, 1);
    //       } else {
    //         candle.setLEDs(255, 0, 0, 0, 4, 1);
    //       }
    //       // cams are disconnected
    //     }
    //     lastAprilTagCams[i] = aprilTagCams[i];
    //   }
    // }

    if (DriverStation.isEStopped()) {
      // solid red
      candle.setLEDs(255, 0, 0);
    } else if (lowBattery) {
      for (int i = 0; i < NUM_LED_STRIPS; i++) {
        // low battery, red fading
        candle.animate(
            new SingleFadeAnimation(
                255,
                0,
                0,
                0,
                animationSpeed.get(),
                LEDS_PER_STRIP,
                LEDS_PER_STRIP * i + HARDWARE_LEDS),
            i);
      }
    } else if (endGameWarning) {
      for (int i = 0; i < NUM_LED_STRIPS; i++) {
        // flash red
        candle.animate(
            new StrobeAnimation(
                255,
                0,
                255,
                0,
                animationSpeed.get(),
                LEDS_PER_STRIP,
                LEDS_PER_STRIP * i + HARDWARE_LEDS),
            i);
      }
    }
    else if (coralInBot) {
        for (int i = 0; i < NUM_LED_STRIPS; i++) {
          // Solid Green
          candle.animate(
              new RainbowAnimation(
                  animationBrightness.getAsDouble(),
                  animationSpeed.get(),
                  LEDS_PER_STRIP,
                  false,
                  LEDS_PER_STRIP * i + HARDWARE_LEDS),
              i);
        }
    }
    // } else if (ampScore) {
    //   for (int i = 0; i < NUM_LED_STRIPS; i++) {
    //     // amp score, blue chaser
    //     candle.animate(
    //         new LarsonAnimation(
    //             0,
    //             0,
    //             255,
    //             0,
    //             animationSpeed.get(),
    //             LEDS_PER_STRIP,
    //             BounceMode.Front,
    //             4,
    //             LEDS_PER_STRIP * i + HARDWARE_LEDS),
    //         i);
    //   }
    // } else if (aligning) {
    //   for (int i = 0; i < NUM_LED_STRIPS; i++) {
    //     // anytime we are aligning run leds on bot in fire animation
    //     candle.animate(
    //         new FireAnimation(
    //             animationBrightness.getAsDouble(),
    //             animationSpeed.get(),
    //             LEDS_PER_STRIP * 3,
    //             1.0,
    //             0.3,
    //             true,
    //             LEDS_PER_STRIP * i + HARDWARE_LEDS),
    //         i);
    //   }
    // } else if (noteInBot) {
    //   for (int i = 0; i < NUM_LED_STRIPS; i++) {
    //     // rainbow pattern when note staged
    //     candle.animate(
    //         new RainbowAnimation(
    //             animationBrightness.getAsDouble(),
    //             animationSpeed.get(),
    //             LEDS_PER_STRIP,
    //             false,
    //             LEDS_PER_STRIP * i + HARDWARE_LEDS),
    //         i);
    //   }

    // } else if (notePickedUp) {
    //   for (int i = 0; i < NUM_LED_STRIPS; i++) {
    //     // flash green
    //     candle.animate(
    //         new StrobeAnimation(
    //             0,
    //             255,
    //             0,
    //             0,
    //             animationSpeed.get(),
    //             LEDS_PER_STRIP,
    //             LEDS_PER_STRIP * i + HARDWARE_LEDS),
    //         i);
    //   }
    // } else if (seenNote) {
    //   for (int i = 0; i < NUM_LED_STRIPS; i++) {
    //     // flash orange
    //     candle.animate(
    //         new StrobeAnimation(
    //             255,
    //             165,
    //             0,
    //             0,
    //             animationSpeed.get(),
    //             LEDS_PER_STRIP,
    //             LEDS_PER_STRIP * i + HARDWARE_LEDS),
    //         i);
    //   }
    // } else {
    //   isRed =
    //       DriverStation.getAlliance().isPresent()
    //           && DriverStation.getAlliance().get().equals(Alliance.Red);
    //   int redValue = isRed ? 255 : 0;
    //   int blueValue = isRed ? 0 : 255;
    //   // when no pattern run color flow of alliance color
    //   for (int i = 0; i < NUM_LED_STRIPS; i++) {
    //     // color flow alliance color
    //     candle.animate(
    //         new ColorFlowAnimation(
    //             redValue,
    //             0,
    //             blueValue,
    //             0,
    //             animationSpeed.get(),
    //             LEDS_PER_STRIP,
    //             Direction.Backward,
    //             LEDS_PER_STRIP * i + HARDWARE_LEDS),
    //         i);
    //   }
    // }
  }
}
