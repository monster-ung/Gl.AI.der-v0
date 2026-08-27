package de.ungethuem.flugsteuerung

import android.app.Activity
import android.content.Context
import android.graphics.Color
import android.graphics.Typeface
import android.graphics.drawable.GradientDrawable
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.net.wifi.WifiManager
import android.os.Bundle
import android.text.InputType
import android.view.Gravity
import android.view.View
import android.view.WindowManager
import android.widget.Button
import android.widget.EditText
import android.widget.FrameLayout
import android.widget.LinearLayout
import android.widget.ScrollView
import android.widget.TextView
import java.net.DatagramPacket
import java.net.DatagramSocket
import java.net.InetAddress
import java.util.concurrent.Executors
import java.util.concurrent.ScheduledExecutorService
import java.util.concurrent.TimeUnit
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.roundToInt
import kotlin.math.sqrt

/**
 * Hauptaktivität der Gleitflieger-Steuerung.
 *
 * Unterstützt zwei Achsen:
 *  - Roll (Links/Rechts)
 *  - Pitch (Hoch/Runter)
 *
 * Elevon-Mixing findet auf dem Arduino-Empfänger statt.
 *
 * UDP-Paketformat (4 Bytes):
 *  [0] 0xAA       – Magic Byte / Sync
 *  [1] Armed-Flag – 1 = aktiv, 0 = Failsafe/Neutral
 *  [2] Roll       – 0..180 (90 = Neutral)
 *  [3] Pitch      – 0..180 (90 = Neutral)
 */
class MainActivity : Activity(), SensorEventListener {

    private companion object {
        const val DEFAULT_HOST = "192.168.0.1"
        const val DEFAULT_PORT = 5005
        const val NEUTRAL_ANGLE = 90
        const val MIN_ANGLE = 0
        const val MAX_ANGLE = 180
        const val MAX_DEFLECTION = 50
        const val TILT_ROLL_DEFLECTION_DEG = 35f
        const val TILT_PITCH_DEFLECTION_DEG = 25f
        const val SEND_INTERVAL_MS = 50L
        // Expo-Defaults (zur Laufzeit über Trim verstellbar).
        const val EXPO_DEFAULT = 0.6f
        const val EXPO_MIN = 0.0f
        const val EXPO_MAX = 1.0f
        const val EXPO_STEP = 0.1f
    }

    // Aktuelle Expo-Stärke: 0.0 = linear, 1.0 = maximale Abschwächung um die Mitte.
    @Volatile private var expoStrength = EXPO_DEFAULT

    /**
     * Kubische Expo-Kennlinie für normalisierte Stick-Werte in [-1, 1].
     * Mischt linear (1-k)*x mit kubisch k*x^3 — Vorzeichen bleibt erhalten,
     * y(±1) = ±1 unabhängig von k, und die Steigung um 0 beträgt (1-k).
     */
    private fun applyExpo(x: Float, k: Float): Float {
        val clamped = x.coerceIn(-1f, 1f)
        return (1f - k) * clamped + k * clamped * clamped * clamped
    }

    // --- Netzwerk ---
    private val commandLock = Any()
    private val networkExecutor: ScheduledExecutorService = Executors.newSingleThreadScheduledExecutor()
    @Volatile private var targetHost = DEFAULT_HOST
    @Volatile private var targetPort = DEFAULT_PORT
    @Volatile private var armed = false
    private var socket: DatagramSocket? = null
    private var lastUiNetworkUpdate = 0L
    private var packetsSent = 0L
    private var cachedAddress: InetAddress? = null  // Cache für InetAddress (verhindert GC)
    private var cachedHostString = ""  // Track letzten Host für Cache-Invalidierung
    private var wifiLock: WifiManager.WifiLock? = null  // High-Performance WiFi Lock

    // --- Steuerungswerte ---
    private var rollAngle = NEUTRAL_ANGLE
    private var pitchAngle = NEUTRAL_ANGLE
    private var rollNormalized = 0f
    private var pitchNormalized = 0f

    // --- Trim (einstellbare Neutralposition) ---
    private var trimRollNeutral = NEUTRAL_ANGLE    // Roll neutral (default 90)
    private var trimPitchNeutral = NEUTRAL_ANGLE   // Pitch neutral (default 90)

    // --- Modus ---
    private var tiltMode = true
    private var invertRoll = false
    private var invertPitch = false

    // --- Sensoren & Kalibrierung ---
    private lateinit var sensorManager: SensorManager
    // Gyro liefert reine Winkelgeschwindigkeit → immun gegen translatorische Beschleunigung.
    // Der Accelerometer wird nur als Langzeitreferenz benutzt, um Gyro-Drift wegzubekommen,
    // und während eines Schubs (|accel| weit weg von g) komplett ignoriert.
    private var accelerometer: Sensor? = null
    private var gyroscope: Sensor? = null

    // Komplementärfilter-Zustand
    private val accelValues = FloatArray(3)
    private var hasAccelSample = false
    private var lastGyroTimestampNs = 0L
    private var fusedRollDeg = 0f
    private var fusedPitchDeg = 0f
    private var hasFusedInit = false
    // Gewicht des Gyro-Anteils. 0.98 @ ~50 Hz ≈ 1 s Zeitkonstante – schnelle
    // Linearbeschleunigungs-Spikes werden mit nur 2 % gewichtet.
    private val COMPL_ALPHA = 0.98f
    // Earth gravity (m/s²)
    private val GRAVITY = 9.80665f
    // Toleranz für "Accel ist gerade vertrauenswürdig" (|a| nahe g).
    // Außerhalb dieses Bandes wird ausschließlich über das Gyro integriert.
    private val ACCEL_TRUST_BAND = 1.8f

    /**
     * Kalibrierungs-Offsets für die Neutralposition.
     * Werden beim Start oder manuell per "Kalibrieren"-Button gesetzt.
     * Erlaubt das Halten des Handys in ergonomischer Position (~30-45°) als Neutral.
     */
    private var calibRoll = 0f
    private var calibPitch = 0f
    private var isCalibrated = false

    // --- UI-Referenzen ---
    private lateinit var connectionIndicator: View
    private lateinit var titleText: TextView
    private lateinit var ipText: TextView
    private lateinit var portText: TextView
    private lateinit var txCountText: TextView
    private lateinit var lagText: TextView
    private lateinit var modeLabel: TextView
    private lateinit var armButton: Button
    private lateinit var rollServoText: TextView
    private lateinit var pitchServoText: TextView
    private lateinit var modeManualButton: Button
    private lateinit var modeGyroButton: Button
    private lateinit var joystick: JoystickView
    private lateinit var joystickContainer: FrameLayout
    private lateinit var gyroContainer: FrameLayout
    private lateinit var artificialHorizon: ArtificialHorizonView
    private lateinit var calibrateButton: Button
    private lateinit var rollTrimText: TextView
    private lateinit var pitchTrimText: TextView

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
        // Vollbildmodus (Immersive Sticky)
        @Suppress("DEPRECATION")
        window.decorView.systemUiVisibility = (
            View.SYSTEM_UI_FLAG_IMMERSIVE_STICKY
                or View.SYSTEM_UI_FLAG_FULLSCREEN
                or View.SYSTEM_UI_FLAG_HIDE_NAVIGATION
                or View.SYSTEM_UI_FLAG_LAYOUT_STABLE
                or View.SYSTEM_UI_FLAG_LAYOUT_HIDE_NAVIGATION
                or View.SYSTEM_UI_FLAG_LAYOUT_FULLSCREEN
        )

        sensorManager = getSystemService(Context.SENSOR_SERVICE) as SensorManager
        accelerometer = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
        gyroscope = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE)
        
        // WiFi High-Performance Lock erstellen (verhindert Scanning/Power-Saving)
        val wifiManager = applicationContext.getSystemService(Context.WIFI_SERVICE) as WifiManager
        wifiLock = wifiManager.createWifiLock(WifiManager.WIFI_MODE_FULL_HIGH_PERF, "FlightControlLock")

        buildUi()
        startSender()
    }

    override fun onResume() {
        super.onResume()
        // Beide Sensoren registrieren. Falls kein Gyro vorhanden ist, läuft die
        // Steuerung als reiner Accel-Fallback (siehe onSensorChanged).
        accelerometer?.let {
            sensorManager.registerListener(this, it, SensorManager.SENSOR_DELAY_GAME)
        }
        gyroscope?.let {
            sensorManager.registerListener(this, it, SensorManager.SENSOR_DELAY_GAME)
        }
        // WiFi Lock aktivieren für optimale Latenz
        wifiLock?.acquire()
    }

    override fun onPause() {
        setArmed(false)
        sensorManager.unregisterListener(this)
        // WiFi Lock freigeben
        if (wifiLock?.isHeld == true) {
            wifiLock?.release()
        }
        super.onPause()
    }

    override fun onStop() {
        setArmed(false)
        super.onStop()
    }

    override fun onDestroy() {
        setArmed(false)
        networkExecutor.shutdownNow()
        socket?.close()
        socket = null
        // WiFi Lock sicher freigeben
        if (wifiLock?.isHeld == true) {
            wifiLock?.release()
        }
        wifiLock = null
        super.onDestroy()
    }

    // ─────────────────────────────────────────────────────────────
    //  Sensor-Callbacks
    // ─────────────────────────────────────────────────────────────

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) = Unit

    override fun onSensorChanged(event: SensorEvent) {
        if (!tiltMode) return
        when (event.sensor.type) {
            Sensor.TYPE_ACCELEROMETER -> {
                accelValues[0] = event.values[0]
                accelValues[1] = event.values[1]
                accelValues[2] = event.values[2]
                hasAccelSample = true
                // Reiner Accel-Fallback, falls das Gerät kein Gyro besitzt.
                // (Liefert dann das alte Verhalten – besser als gar nichts.)
                if (gyroscope == null) {
                    val ax = accelValues[0]; val ay = accelValues[1]; val az = accelValues[2]
                    val rawRoll = Math.toDegrees(
                        atan2(ay.toDouble(), sqrt((ax * ax + az * az).toDouble()))
                    ).toFloat()
                    val rawPitch = Math.toDegrees(
                        atan2(ax.toDouble(), sqrt((ay * ay + az * az).toDouble()))
                    ).toFloat()
                    emitTiltOutput(rawRoll, rawPitch)
                }
            }

            Sensor.TYPE_GYROSCOPE -> {
                if (!hasAccelSample) return
                val now = event.timestamp
                if (lastGyroTimestampNs == 0L) {
                    lastGyroTimestampNs = now
                    return
                }
                val dt = (now - lastGyroTimestampNs) / 1_000_000_000f
                lastGyroTimestampNs = now
                if (dt <= 0f || dt > 0.2f) return

                // Accel-Referenzwinkel (gleiche Konvention wie ursprünglich).
                val ax = accelValues[0]; val ay = accelValues[1]; val az = accelValues[2]
                val accelRollDeg = Math.toDegrees(
                    atan2(ay.toDouble(), sqrt((ax * ax + az * az).toDouble()))
                ).toFloat()
                val accelPitchDeg = Math.toDegrees(
                    atan2(ax.toDouble(), sqrt((ay * ay + az * az).toDouble()))
                ).toFloat()

                // Erste Initialisierung mit der Accel-Lage, damit wir nicht von 0° starten.
                if (!hasFusedInit) {
                    fusedRollDeg = accelRollDeg
                    fusedPitchDeg = accelPitchDeg
                    hasFusedInit = true
                }

                // Gyro-Integration. Vorzeichen so gewählt, dass d(rawRoll)/dt = +ω_x
                // und d(rawPitch)/dt = -ω_y – abgeleitet aus der atan2-Konvention oben.
                val gyroRollDelta = Math.toDegrees((+event.values[0] * dt).toDouble()).toFloat()
                val gyroPitchDelta = Math.toDegrees((-event.values[1] * dt).toDouble()).toFloat()
                val rollPred = fusedRollDeg + gyroRollDelta
                val pitchPred = fusedPitchDeg + gyroPitchDelta

                // Plausibilitätsprüfung: Nur wenn |a| nahe g liegt, beschreibt der
                // Beschleunigungsvektor wirklich die Erdanziehung. Während eines Schubs
                // ist das nicht der Fall – dann ausschließlich Gyro verwenden.
                val accelMag = sqrt((ax * ax + ay * ay + az * az).toDouble()).toFloat()
                val accelTrustworthy = abs(accelMag - GRAVITY) < ACCEL_TRUST_BAND

                if (accelTrustworthy) {
                    fusedRollDeg = COMPL_ALPHA * rollPred + (1f - COMPL_ALPHA) * accelRollDeg
                    fusedPitchDeg = COMPL_ALPHA * pitchPred + (1f - COMPL_ALPHA) * accelPitchDeg
                } else {
                    fusedRollDeg = rollPred
                    fusedPitchDeg = pitchPred
                }

                emitTiltOutput(fusedRollDeg, fusedPitchDeg)
            }
        }
    }

    /**
     * Übernimmt die fusionierten rohen Pitch-/Roll-Winkel und führt exakt die
     * gleiche Kalibrier-, Normalisierungs-, Invertier- und Kanalzuordnungslogik
     * aus wie die frühere Accelerometer-Implementierung.
     */
    private fun emitTiltOutput(rawRoll: Float, rawPitch: Float) {
        if (!isCalibrated) {
            calibRoll = rawRoll
            calibPitch = rawPitch
            isCalibrated = true
        }

        val rollDeg = rawRoll - calibRoll
        val pitchDeg = rawPitch - calibPitch

        val normRoll = (rollDeg / TILT_ROLL_DEFLECTION_DEG).coerceIn(-1f, 1f)
        val normPitch = (pitchDeg / TILT_PITCH_DEFLECTION_DEG).coerceIn(-1f, 1f)

        val finalRoll = if (invertRoll) normRoll else -normRoll
        val finalPitch = if (invertPitch) normPitch else -normPitch

        // Gleiche Kanalzuordnung wie im Joystick-Listener: roll-Kanal ← -Pitch-Achse, pitch-Kanal ← Roll-Achse
        updateCommand(finalPitch, finalRoll)

        if (::artificialHorizon.isInitialized) {
            runOnUiThread { artificialHorizon.setAttitude(finalRoll, finalPitch) }
        }
    }

    // ─────────────────────────────────────────────────────────────
    //  UI-Aufbau (Infineon Alula Pilot – Landscape)
    // ─────────────────────────────────────────────────────────────

    private fun buildUi() {
        val root = LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
            setBackgroundColor(color(R.color.surface))
        }

        // ─── A. Top-Statusleiste ───
        root.addView(buildStatusBar())

        // ─── Hauptbereich: Links (Info 1/3) | Rechts (Steuerung 2/3) ───
        val mainArea = LinearLayout(this).apply {
            orientation = LinearLayout.HORIZONTAL
            layoutParams = LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.MATCH_PARENT, 0, 1f
            )
            setPadding(dp(8), dp(6), dp(8), dp(6))
        }

        mainArea.addView(buildLeftInfoPanel())
        mainArea.addView(buildRightControlPanel())

        root.addView(mainArea)
        setContentView(root)
    }

    private fun buildStatusBar(): LinearLayout {
        return LinearLayout(this).apply {
            orientation = LinearLayout.HORIZONTAL
            gravity = Gravity.CENTER_VERTICAL
            setBackgroundResource(R.drawable.status_bar_background)
            setPadding(dp(12), dp(6), dp(12), dp(6))
            layoutParams = LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.MATCH_PARENT,
                dp(48)
            )

            // Links: LED + Titel
            connectionIndicator = View(context).apply {
                val size = dp(10)
                layoutParams = LinearLayout.LayoutParams(size, size).also { it.marginEnd = dp(8) }
                background = GradientDrawable().apply {
                    shape = GradientDrawable.OVAL
                    setColor(color(R.color.connection_red))
                }
            }
            addView(connectionIndicator)

            titleText = TextView(context).apply {
                text = "ALULA PILOT"
                textSize = 13f
                setTextColor(color(R.color.text_primary))
                typeface = Typeface.create("sans-serif", Typeface.BOLD)
                letterSpacing = 0.06f
            }
            addView(titleText)

            // Flexible Mitte: IP:Port + Stats
            val centerFlex = LinearLayout(context).apply {
                orientation = LinearLayout.HORIZONTAL
                gravity = Gravity.CENTER
                layoutParams = LinearLayout.LayoutParams(0, LinearLayout.LayoutParams.WRAP_CONTENT, 1f)
                setPadding(dp(12), 0, dp(12), 0)
            }

            ipText = TextView(context).apply {
                text = targetHost
                textSize = 11f
                setTextColor(color(R.color.text_secondary))
                typeface = Typeface.MONOSPACE
                setPadding(dp(6), dp(2), dp(6), dp(2))
                setBackgroundColor(color(R.color.surface_elevated))
                setOnClickListener { showEditDialog("Server IP", targetHost) { targetHost = it.ifEmpty { DEFAULT_HOST }; ipText.text = targetHost } }
            }
            centerFlex.addView(ipText)

            centerFlex.addView(TextView(context).apply {
                text = ":"
                textSize = 11f
                setTextColor(color(R.color.text_secondary))
                setPadding(dp(2), 0, dp(2), 0)
            })

            portText = TextView(context).apply {
                text = targetPort.toString()
                textSize = 11f
                setTextColor(color(R.color.text_secondary))
                typeface = Typeface.MONOSPACE
                setPadding(dp(6), dp(2), dp(6), dp(2))
                setBackgroundColor(color(R.color.surface_elevated))
                setOnClickListener { showEditDialog("Port", targetPort.toString()) { targetPort = it.toIntOrNull()?.takeIf { p -> p in 1..65535 } ?: DEFAULT_PORT; portText.text = targetPort.toString() } }
            }
            centerFlex.addView(portText)

            // Separator
            centerFlex.addView(View(context).apply {
                layoutParams = LinearLayout.LayoutParams(dp(12), 0)
            })

            txCountText = TextView(context).apply {
                text = "TX: 0"
                textSize = 10f
                setTextColor(color(R.color.text_secondary))
                typeface = Typeface.MONOSPACE
            }
            centerFlex.addView(txCountText)

            centerFlex.addView(View(context).apply {
                layoutParams = LinearLayout.LayoutParams(dp(8), 0)
            })

            lagText = TextView(context).apply {
                text = "Lag: --"
                textSize = 10f
                setTextColor(color(R.color.text_secondary))
                typeface = Typeface.MONOSPACE
            }
            centerFlex.addView(lagText)

            centerFlex.addView(View(context).apply {
                layoutParams = LinearLayout.LayoutParams(dp(8), 0)
            })

            modeLabel = TextView(context).apply {
                text = "GYRO"
                textSize = 10f
                setTextColor(color(R.color.accent))
                typeface = Typeface.create("sans-serif", Typeface.BOLD)
            }
            centerFlex.addView(modeLabel)
            addView(centerFlex)

            // Rechts: ARM Button (prominent, immer sichtbar)
            armButton = Button(context).apply {
                text = "ARM"
                textSize = 13f
                typeface = Typeface.DEFAULT_BOLD
                setTextColor(Color.WHITE)
                minWidth = dp(90)
                minimumWidth = dp(90)
                minHeight = dp(36)
                minimumHeight = dp(36)
                setPadding(dp(16), dp(4), dp(16), dp(4))
                background = GradientDrawable().apply {
                    setColor(color(R.color.accent))
                    cornerRadius = dp(12).toFloat()
                }
                setOnClickListener {
                    setArmed(!armed)
                    text = if (armed) "DISARM" else "ARM"
                    background = GradientDrawable().apply {
                        setColor(if (armed) color(R.color.armed_green) else color(R.color.accent))
                        cornerRadius = dp(12).toFloat()
                    }
                }
            }
            addView(armButton)
        }
    }

    private fun buildLeftInfoPanel(): ScrollView {
        return ScrollView(this).apply {
            layoutParams = LinearLayout.LayoutParams(0, LinearLayout.LayoutParams.MATCH_PARENT, 0.33f).also {
                it.marginEnd = dp(6)
            }
            setBackgroundResource(R.drawable.panel_background)
            isVerticalScrollBarEnabled = false
            isFillViewport = true

            val content = LinearLayout(context).apply {
                orientation = LinearLayout.VERTICAL
                layoutParams = FrameLayout.LayoutParams(
                    FrameLayout.LayoutParams.MATCH_PARENT,
                    FrameLayout.LayoutParams.WRAP_CONTENT
                )
                setPadding(dp(14), dp(10), dp(14), dp(10))

                // Servo-Daten Header
                addView(TextView(context).apply {
                    text = "LIVE SERVO DATA"
                    textSize = 10f
                    setTextColor(color(R.color.accent))
                    typeface = Typeface.create("sans-serif", Typeface.BOLD)
                    letterSpacing = 0.15f
                    setPadding(0, 0, 0, dp(6))
                })

                // Servo values in compact horizontal rows
                addView(servoRow("Roll", "90°").also { rollServoText = it.getChildAt(1) as TextView })
                addView(servoRow("Pitch", "90°").also { pitchServoText = it.getChildAt(1) as TextView })

                // Mode Toggle
                addView(TextView(context).apply {
                    text = "MODUS"
                    textSize = 10f
                    setTextColor(color(R.color.accent))
                    typeface = Typeface.create("sans-serif", Typeface.BOLD)
                    letterSpacing = 0.12f
                    setPadding(0, dp(10), 0, dp(6))
                })

                val toggleContainer = LinearLayout(context).apply {
                    orientation = LinearLayout.HORIZONTAL
                    layoutParams = LinearLayout.LayoutParams(
                        LinearLayout.LayoutParams.MATCH_PARENT,
                        LinearLayout.LayoutParams.WRAP_CONTENT
                    )
                }

                modeManualButton = Button(context).apply {
                    text = "MANUELL"
                    textSize = 11f
                    typeface = Typeface.DEFAULT_BOLD
                    setTextColor(color(R.color.text_secondary))
                    setBackgroundResource(R.drawable.toggle_inactive)
                    minHeight = dp(38)
                    minimumHeight = dp(38)
                    setPadding(dp(4), dp(4), dp(4), dp(4))
                    layoutParams = LinearLayout.LayoutParams(0, LinearLayout.LayoutParams.WRAP_CONTENT, 1f).also {
                        it.marginEnd = dp(3)
                    }
                    setOnClickListener { switchMode(false) }
                }
                toggleContainer.addView(modeManualButton)

                modeGyroButton = Button(context).apply {
                    text = "GYRO"
                    textSize = 11f
                    typeface = Typeface.DEFAULT_BOLD
                    setTextColor(color(R.color.text_primary))
                    setBackgroundResource(R.drawable.toggle_active)
                    minHeight = dp(38)
                    minimumHeight = dp(38)
                    setPadding(dp(4), dp(4), dp(4), dp(4))
                    layoutParams = LinearLayout.LayoutParams(0, LinearLayout.LayoutParams.WRAP_CONTENT, 1f).also {
                        it.marginStart = dp(3)
                    }
                    setOnClickListener { switchMode(true) }
                }
                toggleContainer.addView(modeGyroButton)
                addView(toggleContainer)

                // ─── Trim Section ───
                addView(TextView(context).apply {
                    text = "TRIM"
                    textSize = 10f
                    setTextColor(color(R.color.accent))
                    typeface = Typeface.create("sans-serif", Typeface.BOLD)
                    letterSpacing = 0.12f
                    setPadding(0, dp(10), 0, dp(6))
                })

                addView(buildTrimRow("R", trimRollNeutral) { value ->
                    trimRollNeutral = value
                    updateCommand(rollNormalized, pitchNormalized)
                }.also { rollTrimText = it.findViewWithTag("trimValue") })

                addView(buildTrimRow("P", trimPitchNeutral) { value ->
                    trimPitchNeutral = value
                    updateCommand(rollNormalized, pitchNormalized)
                }.also { pitchTrimText = it.findViewWithTag("trimValue") })

                // Expo-Trim: feinere/grobere Kontrolle um die Mitte (0.00 = linear, 1.00 = max).
                addView(buildExpoTrimRow("E", expoStrength) { value ->
                    expoStrength = value
                    updateCommand(rollNormalized, pitchNormalized)
                })
            }
            addView(content)
        }
    }

    private fun servoRow(label: String, value: String): LinearLayout {
        return LinearLayout(this).apply {
            orientation = LinearLayout.HORIZONTAL
            gravity = Gravity.CENTER_VERTICAL
            layoutParams = LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.MATCH_PARENT,
                LinearLayout.LayoutParams.WRAP_CONTENT
            ).also { it.bottomMargin = dp(2) }

            addView(TextView(context).apply {
                text = label
                textSize = 12f
                setTextColor(color(R.color.text_secondary))
                typeface = Typeface.create("sans-serif", Typeface.BOLD)
                layoutParams = LinearLayout.LayoutParams(dp(24), LinearLayout.LayoutParams.WRAP_CONTENT)
            })

            addView(TextView(context).apply {
                text = value
                textSize = 24f
                setTextColor(color(R.color.text_primary))
                typeface = Typeface.create("sans-serif-light", Typeface.NORMAL)
            })
        }
    }

    private fun buildTrimRow(label: String, initialValue: Int, onChange: (Int) -> Unit): LinearLayout {
        var current = initialValue
        return LinearLayout(this).apply {
            orientation = LinearLayout.HORIZONTAL
            gravity = Gravity.CENTER_VERTICAL
            layoutParams = LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.MATCH_PARENT,
                LinearLayout.LayoutParams.WRAP_CONTENT
            ).also { it.bottomMargin = dp(4) }

            // Label
            addView(TextView(context).apply {
                text = label
                textSize = 11f
                setTextColor(color(R.color.text_secondary))
                typeface = Typeface.create("sans-serif", Typeface.BOLD)
                layoutParams = LinearLayout.LayoutParams(dp(20), LinearLayout.LayoutParams.WRAP_CONTENT)
            })

            // Minus button
            addView(Button(context).apply {
                text = "−"
                textSize = 16f
                typeface = Typeface.DEFAULT_BOLD
                setTextColor(color(R.color.text_primary))
                setBackgroundResource(R.drawable.toggle_inactive)
                minWidth = dp(32)
                minimumWidth = dp(32)
                minHeight = dp(32)
                minimumHeight = dp(32)
                setPadding(0, 0, 0, 0)
                layoutParams = LinearLayout.LayoutParams(dp(32), dp(32)).also {
                    it.marginEnd = dp(4)
                }
                setOnClickListener {
                    current = (current - 1).coerceIn(MIN_ANGLE, MAX_ANGLE)
                    onChange(current)
                    (parent as LinearLayout).findViewWithTag<TextView>("trimValue").text = "${current}°"
                }
            })

            // Value (tappable to type)
            addView(TextView(context).apply {
                tag = "trimValue"
                text = "${current}°"
                textSize = 16f
                setTextColor(color(R.color.text_primary))
                typeface = Typeface.MONOSPACE
                gravity = Gravity.CENTER
                layoutParams = LinearLayout.LayoutParams(dp(48), LinearLayout.LayoutParams.WRAP_CONTENT)
                setOnClickListener {
                    showEditDialog("Trim $label", current.toString()) { input ->
                        val parsed = input.toIntOrNull()?.coerceIn(MIN_ANGLE, MAX_ANGLE) ?: current
                        current = parsed
                        onChange(current)
                        text = "${current}°"
                    }
                }
            })

            // Plus button
            addView(Button(context).apply {
                text = "+"
                textSize = 16f
                typeface = Typeface.DEFAULT_BOLD
                setTextColor(color(R.color.text_primary))
                setBackgroundResource(R.drawable.toggle_inactive)
                minWidth = dp(32)
                minimumWidth = dp(32)
                minHeight = dp(32)
                minimumHeight = dp(32)
                setPadding(0, 0, 0, 0)
                layoutParams = LinearLayout.LayoutParams(dp(32), dp(32)).also {
                    it.marginStart = dp(4)
                }
                setOnClickListener {
                    current = (current + 1).coerceIn(MIN_ANGLE, MAX_ANGLE)
                    onChange(current)
                    (parent as LinearLayout).findViewWithTag<TextView>("trimValue").text = "${current}°"
                }
            })
        }
    }

    /** Float-Variante von buildTrimRow für Werte in [EXPO_MIN, EXPO_MAX] mit Schrittweite EXPO_STEP. */
    private fun buildExpoTrimRow(label: String, initialValue: Float, onChange: (Float) -> Unit): LinearLayout {
        var current = initialValue.coerceIn(EXPO_MIN, EXPO_MAX)
        fun format(v: Float) = String.format("%.2f", v)
        return LinearLayout(this).apply {
            orientation = LinearLayout.HORIZONTAL
            gravity = Gravity.CENTER_VERTICAL
            layoutParams = LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.MATCH_PARENT,
                LinearLayout.LayoutParams.WRAP_CONTENT
            ).also { it.bottomMargin = dp(4) }

            addView(TextView(context).apply {
                text = label
                textSize = 11f
                setTextColor(color(R.color.text_secondary))
                typeface = Typeface.create("sans-serif", Typeface.BOLD)
                layoutParams = LinearLayout.LayoutParams(dp(20), LinearLayout.LayoutParams.WRAP_CONTENT)
            })

            addView(Button(context).apply {
                text = "−"
                textSize = 16f
                typeface = Typeface.DEFAULT_BOLD
                setTextColor(color(R.color.text_primary))
                setBackgroundResource(R.drawable.toggle_inactive)
                minWidth = dp(32); minimumWidth = dp(32)
                minHeight = dp(32); minimumHeight = dp(32)
                setPadding(0, 0, 0, 0)
                layoutParams = LinearLayout.LayoutParams(dp(32), dp(32)).also { it.marginEnd = dp(4) }
                setOnClickListener {
                    current = (current - EXPO_STEP).coerceIn(EXPO_MIN, EXPO_MAX)
                    onChange(current)
                    (parent as LinearLayout).findViewWithTag<TextView>("expoValue").text = format(current)
                }
            })

            addView(TextView(context).apply {
                tag = "expoValue"
                text = format(current)
                textSize = 16f
                setTextColor(color(R.color.text_primary))
                typeface = Typeface.MONOSPACE
                gravity = Gravity.CENTER
                layoutParams = LinearLayout.LayoutParams(dp(48), LinearLayout.LayoutParams.WRAP_CONTENT)
                setOnClickListener {
                    showEditDialog("Expo $label", format(current)) { input ->
                        val parsed = input.replace(',', '.').toFloatOrNull()?.coerceIn(EXPO_MIN, EXPO_MAX) ?: current
                        current = parsed
                        onChange(current)
                        text = format(current)
                    }
                }
            })

            addView(Button(context).apply {
                text = "+"
                textSize = 16f
                typeface = Typeface.DEFAULT_BOLD
                setTextColor(color(R.color.text_primary))
                setBackgroundResource(R.drawable.toggle_inactive)
                minWidth = dp(32); minimumWidth = dp(32)
                minHeight = dp(32); minimumHeight = dp(32)
                setPadding(0, 0, 0, 0)
                layoutParams = LinearLayout.LayoutParams(dp(32), dp(32)).also { it.marginStart = dp(4) }
                setOnClickListener {
                    current = (current + EXPO_STEP).coerceIn(EXPO_MIN, EXPO_MAX)
                    onChange(current)
                    (parent as LinearLayout).findViewWithTag<TextView>("expoValue").text = format(current)
                }
            })
        }
    }

    private fun buildRightControlPanel(): FrameLayout {
        return FrameLayout(this).apply {
            layoutParams = LinearLayout.LayoutParams(0, LinearLayout.LayoutParams.MATCH_PARENT, 0.67f)
            setBackgroundResource(R.drawable.panel_background)
            setPadding(dp(10), dp(10), dp(10), dp(10))

            // ─── Manueller Modus: Joystick ───
            joystickContainer = FrameLayout(context).apply {
                visibility = if (tiltMode) View.GONE else View.VISIBLE
                layoutParams = FrameLayout.LayoutParams(
                    FrameLayout.LayoutParams.MATCH_PARENT,
                    FrameLayout.LayoutParams.MATCH_PARENT
                )
            }
            joystick = JoystickView(
                context,
                color(R.color.joystick_base),
                color(R.color.joystick_thumb),
                color(R.color.joystick_ring)
            ).apply {
                layoutParams = FrameLayout.LayoutParams(
                    FrameLayout.LayoutParams.MATCH_PARENT,
                    FrameLayout.LayoutParams.MATCH_PARENT
                )
                listener = object : JoystickView.Listener {
                    override fun onJoystickMoved(x: Float, y: Float) {
                        if (!tiltMode) {
                            val finalX = if (invertRoll) -x else x
                            val finalY = if (invertPitch) -y else y
                            updateCommand(finalY, finalX)
                        }
                    }
                }
            }
            joystickContainer.addView(joystick)
            addView(joystickContainer)

            // ─── Gyro-Modus: Künstlicher Horizont + Kalibrieren ───
            gyroContainer = FrameLayout(context).apply {
                visibility = if (tiltMode) View.VISIBLE else View.GONE
                layoutParams = FrameLayout.LayoutParams(
                    FrameLayout.LayoutParams.MATCH_PARENT,
                    FrameLayout.LayoutParams.MATCH_PARENT
                )
            }

            // Horizont zentriert
            artificialHorizon = ArtificialHorizonView(context).apply {
                layoutParams = FrameLayout.LayoutParams(
                    FrameLayout.LayoutParams.MATCH_PARENT,
                    FrameLayout.LayoutParams.MATCH_PARENT
                ).also { it.setMargins(dp(10), dp(10), dp(140), dp(10)) }
            }
            gyroContainer.addView(artificialHorizon)

            // Kalibrieren-Button rechts
            calibrateButton = Button(context).apply {
                text = "NEUTRALPUNKT\nKALIBRIEREN"
                textSize = 12f
                typeface = Typeface.DEFAULT_BOLD
                setTextColor(Color.WHITE)
                gravity = Gravity.CENTER
                setBackgroundResource(R.drawable.calibrate_button)
                layoutParams = FrameLayout.LayoutParams(
                    dp(120),
                    dp(80),
                    Gravity.END or Gravity.CENTER_VERTICAL
                ).also { it.marginEnd = dp(8) }
                setOnClickListener { calibrateSensors() }
            }
            gyroContainer.addView(calibrateButton)

            addView(gyroContainer)
        }
    }

    // ─────────────────────────────────────────────────────────────
    //  Steuerlogik
    // ─────────────────────────────────────────────────────────────

    /**
     * Berechnet Servo-Winkel aus normalisierten Eingaben.
     * @param roll [-1, 1]: -1 = voll links, +1 = voll rechts
     * @param pitch [-1, 1]: -1 = Nase runter, +1 = Nase hoch
     */
    private fun updateCommand(roll: Float, pitch: Float) {
        val clampedRoll = roll.coerceIn(-1f, 1f)
        val clampedPitch = pitch.coerceIn(-1f, 1f)

        // Expo-Kennlinie: feinere Kontrolle um die Mitte, voller Endausschlag bleibt erhalten.
        val expoRoll = applyExpo(clampedRoll, expoStrength)
        val expoPitch = applyExpo(clampedPitch, expoStrength)

        // Rohe Achsenwerte – Elevon-Mixing erfolgt auf dem Arduino
        val nextRoll = (trimRollNeutral + expoRoll * MAX_DEFLECTION).roundToInt().coerceIn(MIN_ANGLE, MAX_ANGLE)
        val nextPitch = (trimPitchNeutral + expoPitch * MAX_DEFLECTION).roundToInt().coerceIn(MIN_ANGLE, MAX_ANGLE)

        synchronized(commandLock) {
            // Roh-Werte speichern, damit Re-Feeds (z. B. nach Trim-Änderung) Expo nicht doppelt anwenden.
            rollNormalized = clampedRoll
            pitchNormalized = clampedPitch
            rollAngle = nextRoll
            pitchAngle = nextPitch
        }

        // UI-Update der Servo-Anzeige
        runOnUiThread {
            if (::rollServoText.isInitialized) {
                rollServoText.text = "${nextRoll}°"
                pitchServoText.text = "${nextPitch}°"
            }
        }
    }

    /**
     * Setzt die aktuelle Handy-Position als Neutralreferenz.
     * Erlaubt ergonomisches Halten (~30-45° Neigung).
     */
    private fun calibrateSensors() {
        isCalibrated = false // Wird beim nächsten Sensor-Event neu gesetzt
        updateCommand(0f, 0f)
        if (::artificialHorizon.isInitialized) {
            artificialHorizon.setAttitude(0f, 0f)
        }
    }

    // ─────────────────────────────────────────────────────────────
    //  Netzwerk & Sicherheit
    // ─────────────────────────────────────────────────────────────

    private fun startSender() {
        networkExecutor.scheduleAtFixedRate({
            if (armed) {
                val payload = synchronized(commandLock) {
                    byteArrayOf(
                        0xAA.toByte(),
                        1,
                        rollAngle.toByte(),
                        pitchAngle.toByte()
                    )
                }
                sendUdp(payload)
                packetsSent++
            }
        }, 0, SEND_INTERVAL_MS, TimeUnit.MILLISECONDS)
    }

    private fun setArmed(enabled: Boolean) {
        val changed = armed != enabled
        armed = enabled
        if (!enabled && changed) {
            sendFailsafeBurst()
        }
        runOnUiThread { updateConnectionIndicator() }
    }

    private fun sendFailsafeBurst() {
        networkExecutor.execute {
            repeat(5) {
                sendUdp(byteArrayOf(
                    0xAA.toByte(),
                    0,
                    trimRollNeutral.toByte(),
                    trimPitchNeutral.toByte()
                ))
                try {
                    Thread.sleep(30)
                } catch (_: InterruptedException) {
                    return@execute
                }
            }
        }
    }

    private fun sendUdp(payload: ByteArray) {
        try {
            // InetAddress cachen um GC-Pauses zu vermeiden
            if (cachedAddress == null || cachedHostString != targetHost) {
                cachedAddress = InetAddress.getByName(targetHost)
                cachedHostString = targetHost
            }
            
            val packet = DatagramPacket(payload, payload.size, cachedAddress, targetPort)
            val datagramSocket = socket ?: DatagramSocket().also { socket = it }
            val sendStart = System.currentTimeMillis()
            datagramSocket.send(packet)
            val lag = System.currentTimeMillis() - sendStart

            val now = System.currentTimeMillis()
            if (now - lastUiNetworkUpdate > 200L) {
                lastUiNetworkUpdate = now
                runOnUiThread {
                    if (::txCountText.isInitialized) {
                        txCountText.text = "TX: $packetsSent pkts"
                        lagText.text = "Lag: ${lag} ms"
                    }
                }
            }
        } catch (exception: Exception) {
            // Cache invalidieren bei Fehler
            cachedAddress = null
            runOnUiThread {
                if (::connectionIndicator.isInitialized) {
                    (connectionIndicator.background as? GradientDrawable)?.setColor(color(R.color.connection_red))
                }
            }
        }
    }

    private fun updateConnectionIndicator() {
        if (!::connectionIndicator.isInitialized) return
        runOnUiThread {
            val color = if (armed) color(R.color.armed_green) else color(R.color.connection_red)
            (connectionIndicator.background as? GradientDrawable)?.setColor(color)
            if (::armButton.isInitialized) {
                armButton.text = if (armed) "DISARM" else "ARM"
                armButton.background = GradientDrawable().apply {
                    setColor(if (armed) color(R.color.armed_green) else color(R.color.accent))
                    cornerRadius = dp(12).toFloat()
                }
            }
        }
    }

    private fun switchMode(gyro: Boolean) {
        tiltMode = gyro
        joystickContainer.visibility = if (gyro) View.GONE else View.VISIBLE
        gyroContainer.visibility = if (gyro) View.VISIBLE else View.GONE
        modeGyroButton.setBackgroundResource(if (gyro) R.drawable.toggle_active else R.drawable.toggle_inactive)
        modeGyroButton.setTextColor(color(if (gyro) R.color.text_primary else R.color.text_secondary))
        modeManualButton.setBackgroundResource(if (!gyro) R.drawable.toggle_active else R.drawable.toggle_inactive)
        modeManualButton.setTextColor(color(if (!gyro) R.color.text_primary else R.color.text_secondary))
        modeLabel.text = if (gyro) "GYRO" else "MANUELL"
        if (!gyro) {
            updateCommand(joystick.getNormalizedY(), joystick.getNormalizedX())
        }
    }

    private fun showEditDialog(title: String, currentValue: String, onResult: (String) -> Unit) {
        val editText = EditText(this).apply {
            setText(currentValue)
            inputType = if (title.contains("Port")) InputType.TYPE_CLASS_NUMBER else InputType.TYPE_CLASS_TEXT
            setTextColor(color(R.color.text_primary))
            setBackgroundColor(color(R.color.surface_elevated))
            setPadding(dp(12), dp(8), dp(12), dp(8))
        }
        android.app.AlertDialog.Builder(this, android.R.style.Theme_Material_Dialog_Alert)
            .setTitle(title)
            .setView(editText)
            .setPositiveButton("OK") { _, _ -> onResult(editText.text.toString().trim()) }
            .setNegativeButton("Abbrechen", null)
            .show()
    }

    // ─────────────────────────────────────────────────────────────
    //  UI-Hilfsfunktionen
    // ─────────────────────────────────────────────────────────────

    private fun dp(value: Int): Int = (value * resources.displayMetrics.density).roundToInt()

    /**
     * API-level-safe color lookup. Activity.getColor() requires API 23;
     * resources.getColor(int) works on API 22 and below.
     */
    @Suppress("DEPRECATION")
    private fun color(colorRes: Int): Int = resources.getColor(colorRes)
}
