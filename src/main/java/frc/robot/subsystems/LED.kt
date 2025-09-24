package frc.robot.subsystems

import edu.wpi.first.wpilibj.AddressableLED
import edu.wpi.first.wpilibj.AddressableLEDBuffer
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.utils.RobotParameters.ElevatorParameters.elevatorToBeSetState
import frc.robot.utils.RobotParameters.LEDValues
import frc.robot.utils.RobotParameters.LiveRobotValues
import frc.robot.utils.emu.ElevatorState
import frc.robot.utils.emu.LEDState
import xyz.malefic.frc.pingu.LogPingu.log
import xyz.malefic.frc.pingu.LogPingu.logs
import java.util.Random
import kotlin.math.max
import kotlin.math.pow
import kotlin.math.sin

object LED : SubsystemBase() {
    // Laser Effect Properties
    private const val LASER_COUNT = 10
    private const val SPACING = 5

    // LED Hardware Components
    private val leds: AddressableLED = AddressableLED(9)
    private val ledBuffer: AddressableLEDBuffer = AddressableLEDBuffer(LEDValues.LED_COUNT)
    private val brightnessLevels: IntArray = IntArray(LEDValues.LED_COUNT)

    // Animation Controls
    var ledState: LEDState = LEDState.RAINBOW_FLOW
    private var rainbowFirstHue = 0
    private var position = 0
    private var goingForward = true
    private val rand = Random()
    private val robonautLEDTimer = Timer()

    private val laserPositions = ArrayList<Int?>()

    /**
     * Creates a new instance of this LEDSubsystem. This constructor is private since this class is a
     * Singleton. Code should use the [.getInstance] method to get the singleton instance.
     */
    init {
        leds.setLength(ledBuffer.length)
        leds.setData(ledBuffer)
        leds.start()

//        println("LED init")

        robonautLEDTimer.start()

        for (i in 0..<LASER_COUNT) {
            laserPositions.add(-i * SPACING)
        }
    }

    /**
     * This method will be called once per scheduler run. Updates the LED pattern based on the robot
     * state.
     */
    override fun periodic() {
//        println("periodic LED")
        // Enabled Robot

        if (DriverStation.isEnabled()) {
            when (elevatorToBeSetState) {
                ElevatorState.DEFAULT -> ledState = LEDState.RAINBOW_FLOW
                ElevatorState.L1 -> ledState = LEDState.RED_WAVE
                ElevatorState.L2 -> ledState = LEDState.ORANGE_WAVE
                ElevatorState.L3 -> ledState = LEDState.YELLOW_WAVE
                ElevatorState.L4 -> ledState = LEDState.BLUE_WAVE
                else -> {}
            }
        }

        // Disabled Robot (we can do whatever we want)
        if (DriverStation.isDisabled() && !LiveRobotValues.lowBattery) {
            ledState = LEDState.RAINBOW_FLOW
        } else if (LiveRobotValues.lowBattery && DriverStation.isDisabled()) {
            ledState = LEDState.TWINKLE
        }

        when (this.ledState) {
            LEDState.RAINBOW_FLOW -> flowingRainbow()
            LEDState.HIGHTIDE_FLOW -> highTideFlow()
            LEDState.TWINKLE -> twinkle()
            LEDState.ROBONAUT -> robonautsLED()
            LEDState.FUNNY_ROBONAUT -> funnyRobonaunts()
            LEDState.LASER -> laserBeam()
            LEDState.ORANGE_WAVE -> setRGB(255, 255, 0)
            LEDState.RED_WAVE -> setRGB(255, 0, 0)
            LEDState.GREEN_WAVE -> greenWave()
            LEDState.BLUE_WAVE -> setRGB(0, 0, 255)
            LEDState.YELLOW_WAVE -> setRGB(0, 255, 255)
            LEDState.PURPLE_WAVE -> purpleWave()
            else -> setRed()
        }

        log("LED state", this.ledState)
    }

    /**
     * Sets the color for each of the LEDs based on RGB values.
     *
     * @param r (Red) Integer values between 0 - 255
     * @param g (Green) Integer values between 0 - 255
     * @param b (Blue) Integer values between 0 - 255
     */
    fun setRGB(
        r: Int,
        g: Int,
        b: Int,
    ) {
        for (i in 0..<ledBuffer.length) {
            ledBuffer.setRGB(i, r, g, b)
        }
        logs {
            log("LED Length", ledBuffer.length)
            log("LED Color Blue", ledBuffer.getLED(0).blue)
            log("LED Color Red", ledBuffer.getLED(0).red)
            log("LED Color Green", ledBuffer.getLED(0).green)
        }
        leds.setData(ledBuffer)
    }

    /**
     * Sets the color for each of the LEDs based on HSV values
     *
     * @param h (Hue) Integer values between 0 - 180
     * @param s (Saturation) Integer values between 0 - 255
     * @param v (Value) Integer values between 0 - 255
     */
    fun rainbowHSV(
        h: Int,
        s: Int,
        v: Int,
    ) {
        for (i in 0..<ledBuffer.length) {
            ledBuffer.setHSV(i, h, s, v)
        }
        leds.setData(ledBuffer)
    }

    /** Sets the LED color to tan.  */
    fun setTan() {
        setRGB(255, 122, 20)
    }

    /** Sets the LED color to red.  */
    fun setRed() {
        setRGB(255, 0, 0)
    }

    /** Sets the LED color to green.  */
    fun setGreen() {
        setRGB(0, 255, 0)
    }

    /**
     * Sets the LED color to orange. This is a specific shade of orange that is used for the LED
     * strip.
     */
    fun setOrange() {
        setRGB(255, 165, 0)
    }

    /** Sets the LED color to purple.  */
    fun setPurpleColor() {
        setRGB(160, 32, 240)
    }

    /** Sets the LED color to high tide (a specific shade of blue-green).  */
    fun setHighTide() {
        setRGB(0, 182, 174)
    }

    /**
     * Creates a flowing high tide effect on the LED strip. The effect is based on a sine wave pattern
     * that changes over time.
     */
    fun highTideFlow() {
        val currentTime = System.currentTimeMillis()
        val length = ledBuffer.length

        val waveSpeed = 30
        val waveWidth = 55

        for (i in 0..<length) {
            var wave = sin((i + (currentTime.toDouble() / waveSpeed)) % length * (2 * Math.PI / waveWidth))

            wave = (wave + 1) / 2

            val r = (wave * 0).toInt()
            val g = (wave * 200).toInt()
            val b = (wave * 50).toInt()

            ledBuffer.setRGB(i, r, g, b)
        }
        leds.setData(ledBuffer)
    }

    /** Creates a flowing rainbow effect.  */
    fun flowingRainbow() {
        for (i in 0..<ledBuffer.length) {
            val hue = (rainbowFirstHue + (i * 180 / ledBuffer.length)) % 180
            ledBuffer.setHSV(i, hue, 255, 255)
        }

        rainbowFirstHue = rainbowFirstHue + 4 % 180

        leds.setData(ledBuffer)
    }

    /** Creates a twinkle where lights flicker at random.  */
    fun twinkle() {
        for (i in 0..<ledBuffer.length) {
            if (rand.nextDouble() < 0.1) {
                brightnessLevels[i] = rand.nextInt(255)
            } else {
                brightnessLevels[i] = max(0, brightnessLevels[i] - 10)
            }

            val hue = rand.nextInt(180)
            ledBuffer.setHSV(i, hue, 255, brightnessLevels[i])
        }
        leds.setData(ledBuffer)
    }

    /** Creates a robonauts themed LED pattern courtesy of CalamityCow  */
    fun robonautsLED() {
        if (robonautLEDTimer.advanceIfElapsed(0.1)) {
            for (i in 0..<ledBuffer.length) {
                val color = rand.nextInt(0, 100)

                if (color < 70) {
                    ledBuffer.setRGB(i, 0, 0, 0)
                } else if (color < 85) {
                    ledBuffer.setRGB(i, 244, 177, 25)
                } else if (color < 100) {
                    ledBuffer.setRGB(i, 255, 255, 255)
                }

                leds.setData(ledBuffer)
            }
        }
    }

    /** Funny robonaunts lights  */
    fun funnyRobonaunts() {
        for (i in 0..<ledBuffer.length) {
            if (i == position) {
                ledBuffer.setHSV(i, 45, 255, 255)
            } else {
                ledBuffer.setHSV(i, 45, 255, 50)
            }
        }

        if (goingForward) {
            position++
            if (position >= ledBuffer.length - 1) goingForward = false
        } else {
            position--
            if (position < 0) goingForward = true
        }
        leds.setData(ledBuffer)
    }

    /** Laser effect for LED lights  */
    fun laserBeam() {
        for (i in 0..<ledBuffer.length) {
            ledBuffer.setHSV(i, 0, 255, 20)
        }

        for (i in laserPositions.indices) {
            val pos = laserPositions[i]!!
            if (pos >= 0 && pos < ledBuffer.length) {
                ledBuffer.setHSV(pos, 0, 255, 0)
            }
            laserPositions[i] = pos + 2

            if (laserPositions[i]!! >= ledBuffer.length) {
                laserPositions[i] = -rand.nextInt(SPACING)
            }
        }
        leds.setData(ledBuffer)
    }

    /**
     * Creates a wave effect on the LED strip. The wave effect is based on a sine wave pattern that
     * changes over time.
     *
     * @param startColor The starting color of the wave
     * @param endColor The ending color of the wave
     * @param wavelength The wavelength of the wave
     * @param cycleDuration The duration of the wave cycle
     */
    fun createWave(
        startColor: Color,
        endColor: Color,
        wavelength: Double,
        cycleDuration: Double,
    ) {
        var phase =
            (1 - ((Timer.getFPGATimestamp() % cycleDuration) / cycleDuration)) * 2.0 * Math.PI
        val phaseStep = (2.0 * Math.PI) / wavelength
        val waveExponent = 2.0

        for (ledIndex in ledBuffer.length - 1 downTo 0) {
            phase += phaseStep

            // Calculate blend ratio between colors (0.0 to 1.0)
            var blendRatio = (sin(phase).pow(waveExponent) + 1.0) / 2.0

            if (blendRatio.isNaN()) {
                blendRatio = (-sin(phase + Math.PI).pow(waveExponent) + 1.0) / 2.0
                if (blendRatio.isNaN()) {
                    blendRatio = 0.5
                }
            }

            // Interpolate RGB values between colors
            val red = ((startColor.red * (1 - blendRatio)) + (endColor.red * blendRatio)).toInt()
            val green = ((startColor.green * (1 - blendRatio)) + (endColor.green * blendRatio)).toInt()
            val blue = ((startColor.blue * (1 - blendRatio)) + (endColor.blue * blendRatio)).toInt()

            ledBuffer.setRGB(ledIndex, red, green, blue)
        }
        leds.setData(ledBuffer)
    }

    /** Creates a solid color effect on the LED strip.  */
    fun solidColor(color: Color?) {
        for (i in 0..<ledBuffer.length) {
            ledBuffer.setLED(i, color)
        }
        leds.setData(ledBuffer)
    }

    /** Creates a wave red themed effect on the LED strip.  */
    fun redWave() {
        createWave(Color.kCrimson, Color.kDarkRed, 30.0, 5.0)
    }

    /** Creates a wave green themed effect on the LED strip.  */
    fun greenWave() {
        createWave(Color.kSpringGreen, Color.kDarkGreen, 30.0, 5.0)
    }

    /** Creates a wave blue themed effect on the LED strip.  */
    fun blueWave() {
        createWave(Color.kAliceBlue, Color.kAquamarine, 30.0, 5.0)
    }

    /** Creates a wave yellow themed effect on the LED strip.  */
    fun yellowWave() {
        createWave(Color.kYellow, Color.kGold, 30.0, 5.0)
    }

    /** Creates a wave purple themed effect on the LED strip.  */
    fun purpleWave() {
        createWave(Color.kViolet, Color.kIndigo, 30.0, 5.0)
    }

    /** Creates a wave orange themed effect on the LED strip.  */
    fun orangeWave() {
        createWave(Color.kOrange, Color.kDarkOrange, 30.0, 5.0)
    }

    /** Creates a wave pink themed effect on the LED strip.  */
    fun pinkWave() {
        createWave(Color.kMagenta, Color.kHotPink, 30.0, 5.0)
    }

    fun setDarkGreen() {
        solidColor(Color.kDarkSeaGreen)
    }

    fun setLightGreen() {
        solidColor(Color.kLimeGreen)
    }
}
