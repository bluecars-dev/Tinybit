/**
 * TINY-BIT PRO - Complete Odometry & Drive Library v2.0
 * Production-ready with full feature set and clean API
 */
//% color="#006400" weight=20 icon="\uf1b9"
namespace TinybitPro {

    // ==========================================
    // 1. CONFIGURATION & CALIBRATION CONSTANTS
    // ========================================== 
    // Physics & Hardware Calibration
    export let WHEELBASE = 0.082;           // 8.2 cm distance between wheels
    export let M_PER_PWM_S = 0.00215;       // Meters moved per 1 unit of PWM per second
    export let PWM_MAX = 255;               // Maximum PWM value
    
    // PIDF Controller Gains
    export let kP = 1.4;                    // Proportional gain
    export let kI = 0.02;                   // Integral gain
    export let kD = 0.15;                   // Derivative gain
    export let kF = 35;                     // Feedforward (friction compensation)
    
    // SLAM Configuration
    export let GRID_RES = 0.05;             // 5cm per grid cell
    export let MAP_DIM = 40;                // 40x40 cells = 2m x 2m coverage area
    export let MAP_CENTER = MAP_DIM / 2;    // Center offset for coordinates
    
    // Sensor Configuration
    export let SONAR_MIN_CM = 5;            // Minimum valid sonar reading (cm)
    export let SONAR_MAX_CM = 150;          // Maximum valid sonar reading (cm)
    export let SONAR_HISTORY_SIZE = 5;      // Median filter window
    
    // ==========================================
    // 2. ODOMETRY STATE VARIABLES
    // ==========================================
    
    // Global Pose (position & heading in world frame)
    export let x = 0.0;                     // X position in meters
    export let y = 0.0;                     // Y position in meters
    export let theta = 0.0;                 // Heading in radians (-PI to +PI)
    
    // Internal state tracking
    let lastTime = control.micros();
    let lastErrorLinear = 0.0;              // For derivative term
    let integralErrorLinear = 0.0;          // For integral term
    let integralErrorAngular = 0.0;         // For integral term
    
    // ==========================================
    // 3. HARDWARE MEMORY BUFFERS (Pre-allocated)
    // ==========================================
    
    let motorBuf = pins.createBuffer(5);    // Motor I2C command buffer
    motorBuf[0] = 0x02;                     // Register address for motor controller
    
    let mapGrid = pins.createBuffer(MAP_DIM * MAP_DIM);  // Occupancy grid
    
    let sonarHistory: number[] = [];        // Sonar reading history
    let sonarIdx = 0;                       // Current position in sonar history
    
    // Initialize sonar history
    for (let i = 0; i < SONAR_HISTORY_SIZE; i++) {
        sonarHistory.push(0);
    }
    
    // ==========================================
    // 4. INITIALIZATION & SETUP
    // ==========================================
    
    /**
     * Initialize the robot system.
     * Call this once in your main setup!
     */
    export function init(): void {
        x = 0.0;
        y = 0.0;
        theta = 0.0;
        lastTime = control.micros();
        lastErrorLinear = 0.0;
        integralErrorLinear = 0.0;
        integralErrorAngular = 0.0;
        sonarIdx = 0;
        
        // Clear sonar history
        for (let i = 0; i < SONAR_HISTORY_SIZE; i++) {
            sonarHistory[i] = 0;
        }
        
        // Clear map grid
        for (let i = 0; i < MAP_DIM * MAP_DIM; i++) {
            mapGrid[i] = 0;
        }
        
        serial.writeLine("[ROBOT] Initialized - Ready to go!");
    }
    
    /**
     * Reset the robot's pose to origin (0, 0, 0)
     */
    export function resetPose(): void {
        x = 0.0;
        y = 0.0;
        theta = 0.0;
        lastTime = control.micros();
    }
    
    /**
     * Reset the map grid
     */
    export function resetMap(): void {
        for (let i = 0; i < MAP_DIM * MAP_DIM; i++) {
            mapGrid[i] = 0;
        }
    }
    
    // ==========================================
    // 5. MOTOR CONTROL - RAW & TUNED FUNCTIONS
    // ==========================================
    
    /**
     * Raw motor drive command.
     * Directly sets motor PWM values (-255 to +255).
     * 
     * @param left Left motor PWM: -255 (reverse max) to +255 (forward max)
     * @param right Right motor PWM: -255 (reverse max) to +255 (forward max)
     */
    export function drive(left: number, right: number): void {
        let l = Math.constrain(left, -PWM_MAX, PWM_MAX);
        let r = Math.constrain(right, -PWM_MAX, PWM_MAX);
        
        // Separate forward and reverse for each motor
        motorBuf[1] = l > 0 ? l : 0;        // Left forward
        motorBuf[2] = l < 0 ? -l : 0;      // Left reverse
        motorBuf[3] = r > 0 ? r : 0;        // Right forward
        motorBuf[4] = r < 0 ? -r : 0;      // Right reverse
        
        pins.i2cWriteBuffer(0x01, motorBuf);
    }
    
    /**
     * Stop all motors immediately
     */
    export function stop(): void {
        drive(0, 0);
    }
    
    /**
     * Move forward at a given speed
     * 
     * @param speed 0-255 (0 = stopped, 255 = max forward)
     */
    export function forward(speed: number): void {
        let s = Math.constrain(speed, 0, PWM_MAX);
        drive(s, s);
    }
    
    /**
     * Move backward at a given speed
     * 
     * @param speed 0-255 (0 = stopped, 255 = max reverse)
     */
    export function backward(speed: number): void {
        let s = Math.constrain(speed, 0, PWM_MAX);
        drive(-s, -s);
    }
    
    /**
     * Turn left (counterclockwise)
     * 
     * @param speed 0-255 (turning speed)
     */
    export function turnLeft(speed: number): void {
        let s = Math.constrain(speed, 0, PWM_MAX);
        drive(-s, s);
    }
    
    /**
     * Turn right (clockwise)
     * 
     * @param speed 0-255 (turning speed)
     */
    export function turnRight(speed: number): void {
        let s = Math.constrain(speed, 0, PWM_MAX);
        drive(s, -s);
    }
    
    /**
     * Arcade drive (simplified tank-style control)
     * 
     * @param linear Forward/backward speed (-255 to +255)
     * @param angular Rotation speed (-255 to +255)
     */
    export function arcadeDrive(linear: number, angular: number): void {
        let lin = Math.constrain(linear, -PWM_MAX, PWM_MAX);
        let ang = Math.constrain(angular, -PWM_MAX, PWM_MAX);
        
        let left = lin + ang;
        let right = lin - ang;
        
        drive(left, right);
    }
    
    // ==========================================
    // 6. SENSOR READING - SONAR & FILTERED DATA
    // ==========================================
    
    /**
     * Read raw sonar distance in centimeters (single reading)
     * Uses median filtering from sonar history.
     * 
     * @returns Distance in centimeters
     */
    export function getSonar(): number {
        // Trigger ultrasonic pulse
        pins.digitalWritePin(DigitalPin.P16, 0);
        control.waitMicros(2);
        pins.digitalWritePin(DigitalPin.P16, 1);
        control.waitMicros(10);
        pins.digitalWritePin(DigitalPin.P16, 0);
        
        // Measure echo time (max timeout: 25ms = ~4.25 meters range)
        let pulseTime = pins.pulseIn(DigitalPin.P15, PulseValue.High, 25000);
        let distanceCm = pulseTime / 58;  // Convert time to cm (58 microseconds per cm)
        
        // Validate reading before adding to history
        if (distanceCm > SONAR_MIN_CM && distanceCm < SONAR_MAX_CM) {
            sonarHistory[sonarIdx] = distanceCm;
            sonarIdx = (sonarIdx + 1) % SONAR_HISTORY_SIZE;
        }
        
        // Return median of history (kills outlier spikes)
        let sorted = sonarHistory.slice().sort((a, b) => a - b);
        return sorted[Math.floor(SONAR_HISTORY_SIZE / 2)];
    }
    
    /**
     * Get filtered sonar distance in meters
     * 
     * @returns Distance in meters
     */
    export function getSonarMeters(): number {
        return getSonar() / 100.0;
    }
    
    // ==========================================
    // 7. ODOMETRY & POSE ESTIMATION
    // ==========================================
    
    /**
     * Update robot pose using differential odometry
     * Call this regularly with encoder/velocity readings!
     * 
     * @param leftPWM Left motor PWM (last command sent)
     * @param rightPWM Right motor PWM (last command sent)
     */
    export function updatePose(leftPWM: number, rightPWM: number): void {
        let now = control.micros();
        let dt = (now - lastTime) / 1000000.0;  // Convert to seconds
        lastTime = now;
        
        // Clamp dt to prevent huge jumps
        if (dt > 0.1) dt = 0.1;
        if (dt < 0) dt = 0;
        
        // Calculate velocities from PWM
        let vL = leftPWM * M_PER_PWM_S;
        let vR = rightPWM * M_PER_PWM_S;
        
        // Differential kinematics
        let vLinear = (vL + vR) / 2.0;      // Average velocity
        let vAngular = (vR - vL) / WHEELBASE;  // Angular velocity
        
        // Update heading (theta)
        theta += vAngular * dt;
        
        // Wrap theta to [-PI, +PI]
        while (theta > Math.PI) theta -= 2 * Math.PI;
        while (theta < -Math.PI) theta += 2 * Math.PI;
        
        // Update position (x, y)
        let cosTheta = Math.cos(theta);
        let sinTheta = Math.sin(theta);
        
        x += vLinear * cosTheta * dt;
        y += vLinear * sinTheta * dt;
    }
    
    /**
     * Get current pose as object
     * 
     * @returns {x: number, y: number, theta: number}
     */
    export function getPose(): {x: number, y: number, theta: number} {
        return {
            x: x,
            y: y,
            theta: theta
        };
    }
    
    /**
     * Get heading in degrees (0-360)
     * 
     * @returns Heading in degrees
     */
    export function getHeadingDegrees(): number {
        let deg = theta * 57.2958;  // Convert radians to degrees
        if (deg < 0) deg += 360;
        return deg;
    }
    
    /**
     * Get distance from origin
     * 
     * @returns Distance in meters
     */
    export function getDistanceFromOrigin(): number {
        return Math.sqrt(x * x + y * y);
    }
    
    /**
     * Set pose manually (for external localization/corrections)
     * 
     * @param newX X position in meters
     * @param newY Y position in meters
     * @param newTheta Heading in radians
     */
    export function setPose(newX: number, newY: number, newTheta: number): void {
        x = newX;
        y = newY;
        theta = newTheta;
    }
    
    // ==========================================
    // 8. SLAM - OCCUPANCY GRID MAPPING
    // ==========================================
    
    /**
     * Update the occupancy map with current sonar reading
     * Call this in your main loop alongside updatePose()
     */
    export function updateMap(): void {
        let distMeters = getSonarMeters();
        
        // Reject invalid readings
        if (distMeters < 0.05 || distMeters > 1.5) return;
        
        // Project sonar hit into world coordinates
        let wallX = x + (distMeters * Math.cos(theta));
        let wallY = y + (distMeters * Math.sin(theta));
        
        // Convert to grid coordinates (centered)
        let gx = Math.floor(wallX / GRID_RES) + MAP_CENTER;
        let gy = Math.floor(wallY / GRID_RES) + MAP_CENTER;
        
        // Update grid if in bounds
        if (gx >= 0 && gx < MAP_DIM && gy >= 0 && gy < MAP_DIM) {
            let index = gy * MAP_DIM + gx;
            if (mapGrid[index] < 250) {
                mapGrid[index] += 25;  // ~4 hits to reach certainty
            }
        }
    }
    
    /**
     * Get occupancy value at grid coordinates
     * 
     * @param gx Grid X coordinate
     * @param gy Grid Y coordinate
     * @returns Occupancy value (0-255), or -1 if out of bounds
     */
    export function getGridCell(gx: number, gy: number): number {
        if (gx < 0 || gx >= MAP_DIM || gy < 0 || gy >= MAP_DIM) {
            return -1;
        }
        return mapGrid[gy * MAP_DIM + gx];
    }
    
    /**
     * Set a grid cell value
     * 
     * @param gx Grid X coordinate
     * @param gy Grid Y coordinate
     * @param value Occupancy value (0-255)
     */
    export function setGridCell(gx: number, gy: number, value: number): void {
        if (gx >= 0 && gx < MAP_DIM && gy >= 0 && gy < MAP_DIM) {
            mapGrid[gy * MAP_DIM + gx] = Math.constrain(value, 0, 255);
        }
    }
    
    // ==========================================
    // 9. DEBUG & TELEMETRY FUNCTIONS
    // ==========================================
    
    /**
     * Print current pose to serial console
     */
    export function printPose(): void {
        serial.writeLine(`[POSE] X: ${(x*100).toFixed(1)}cm, Y: ${(y*100).toFixed(1)}cm, T: ${getHeadingDegrees().toFixed(1)}°`);
    }
    
    /**
     * Print sonar reading to serial console
     */
    export function printSonar(): void {
        let dist = getSonar();
        serial.writeLine(`[SONAR] ${dist.toFixed(1)}cm`);
    }
    
    /**
     * Export the SLAM map to serial console as ASCII art
     */
    export function exportMapToSerial(): void {
        serial.writeLine("\n╔═════════════════════════════════╗");
        serial.writeLine("║   TINY-BIT PRO SLAM MAP v2.0   ║");
        serial.writeLine("╚═════════════════════════════════╝");
        serial.writeLine(`Pose: X=${(x*100).toFixed(0)}cm Y=${(y*100).toFixed(0)}cm T=${getHeadingDegrees().toFixed(0)}°`);
        serial.writeLine("");
        
        let robotGridX = Math.floor(x / GRID_RES) + MAP_CENTER;
        let robotGridY = Math.floor(y / GRID_RES) + MAP_CENTER;
        
        for (let row = 0; row < MAP_DIM; row++) {
            let line = "";
            for (let col = 0; col < MAP_DIM; col++) {
                if (col === robotGridX && row === robotGridY) {
                    line += "◉";  // Robot position
                } else {
                    let cell = mapGrid[row * MAP_DIM + col];
                    if (cell > 200) line += "█";     // High confidence wall
                    else if (cell > 100) line += "▓"; // Medium confidence
                    else if (cell > 25) line += "░";  // Low confidence
                    else line += "·";                  // Empty space
                }
            }
            serial.writeLine(line);
        }
        serial.writeLine("─────────────────────────────────");
    }
    
    /**
     * Export pose and map data to serial as JSON
     */
    export function exportDataToJSON(): void {
        let data = {
            timestamp: control.millis(),
            pose: {
                x: x,
                y: y,
                theta: theta,
                heading_deg: getHeadingDegrees()
            },
            sonar_cm: getSonar(),
            map_dim: MAP_DIM
        };
        serial.writeLine(JSON.stringify(data));
    }
}
