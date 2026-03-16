/**
 * TINY-BIT PRO 
 */
//% color="#006400" weight=20 icon="\uf1b9"
namespace TinybitPro {

    // ==========================================
    // 1. GLOBAL TUNING VARIABLES
    // ==========================================
    
    // Physics Calibration
    export let WHEELBASE = 0.082;        // 8.2 cm distance between wheels
    export let M_PER_PWM_S = 0.00215;    // Meters moved per 1 unit of PWM per second
    
    // PIDF Controller Setup
    export let kP = 1.4, kI = 0.02, kD = 0.15;
    export let kF = 35;                  // Minimum power to overcome wheel friction

    // SLAM Setup
    export let GRID_RES = 0.05;          // Map resolution: 5cm per cell
    export let MAP_DIM = 40;             // Map size: 40x40 cells (2m x 2m area)

    // Odometry State Space
    export let x = 0.0, y = 0.0;
    export let theta = 0.0;              // Heading in Radians
    let lastTime = control.micros();

    // ==========================================
    // 2. OPTIMIZED HARDWARE MEMORY
    // ==========================================
    
    // Memory recycling: Create buffers ONCE to prevent RAM lag
    let motorBuf = pins.createBuffer(5);
    motorBuf[0] = 0x02; // Register address for motors
    
    let mapGrid = pins.createBuffer(MAP_DIM * MAP_DIM);
    
    // Rolling Window array for the Ultrasonic sensor
    let sonarHistory = [0, 0, 0, 0, 0];
    let sonarIdx = 0;


    // ==========================================
    // 3. HIGH-SPEED DRIVERS
    // ==========================================

    /**
     * Zero-Lag Motor Driver.
     * Recycles memory instead of creating new buffers.
     */
    export function drive(left: number, right: number): void {
        let l = Math.constrain(left, -255, 255);
        let r = Math.constrain(right, -255, 255);

        motorBuf[1] = l > 0 ? l : 0;
        motorBuf[2] = l < 0 ? -l : 0;
        motorBuf[3] = r > 0 ? r : 0;
        motorBuf[4] = r < 0 ? -r : 0;

        pins.i2cWriteBuffer(0x01, motorBuf);
    }

    /**
     * Advanced Temporal Sonar.
     * Fires 1 fast ping, returns the Median of the last 5.
     */
    export function getSonar(): number {
        // Fast hardware trigger
        pins.digitalWritePin(DigitalPin.P16, 0);
        control.waitMicros(2);
        pins.digitalWritePin(DigitalPin.P16, 1);
        control.waitMicros(10);
        pins.digitalWritePin(DigitalPin.P16, 0);

        // Max timeout is 25ms to prevent freezing
        let d = pins.pulseIn(DigitalPin.P15, PulseValue.High, 25000) / 58;
        
        // Only accept realistic values into history
        if (d > 2 && d < 400) {
            sonarHistory[sonarIdx] = d;
            sonarIdx = (sonarIdx + 1) % 5;
        }

        // Return the Median to automatically kill "ghost" spikes
        let sorted = sonarHistory.slice().sort();
        return sorted[2];
    }


    // ==========================================
    // 4. THE ROBOT BRAIN (Odom & SLAM)
    // ==========================================

    /**
     * Microsecond Dead Reckoning Kinematics.
     */
    export function updatePose(lp: number, rp: number): void {
        let now = control.micros();
        let dt = (now - lastTime) / 1000000.0;
        lastTime = now;

        // Calculate velocity vectors
        let vL = lp * M_PER_PWM_S;
        let vR = rp * M_PER_PWM_S;
        let vLinear = (vR + vL) / 2.0;
        let vAngular = (vR - vL) / WHEELBASE;

        // Update Global X, Y, and Heading
        theta += vAngular * dt;
        
        // Keep theta wrapped between -PI and +PI for cleaner math
        if (theta > Math.PI) theta -= 2 * Math.PI;
        if (theta < -Math.PI) theta += 2 * Math.PI;

        x += vLinear * Math.cos(theta) * dt;
        y += vLinear * Math.sin(theta) * dt;
    }

    /**
     * Probability Occupancy Grid Mapping.
     */
    export function updateMap(): void {
        let d_meters = getSonar() / 100.0;
        
        // Ignore noise too close or too far
        if (d_meters < 0.05 || d_meters > 1.5) return;

        // Project the sensor hit into global map coordinates
        let wallX = x + (d_meters * Math.cos(theta));
        let wallY = y + (d_meters * Math.sin(theta));

        // Convert coordinates to grid pixels
        let gx = Math.floor(wallX / GRID_RES) + (MAP_DIM / 2);
        let gy = Math.floor(wallY / GRID_RES) + (MAP_DIM / 2);

        // If inside the map, increase the "Wall Probability"
        if (gx >= 0 && gx < MAP_DIM && gy >= 0 && gy < MAP_DIM) {
            let index = gy * MAP_DIM + gx;
            if (mapGrid[index] < 250) {
                mapGrid[index] += 25; // Requires ~4 hits to reach 100% certainty
            }
        }
    }

    /**
     * Prints the 2D Map over the USB Serial Cable.
     */
    export function exportMapToSerial(): void {
        serial.writeLine("\n--- TINY-BIT PRO SLAM MAP ---");
        serial.writeLine(`Current Pose: X:${Math.round(x*100)}cm, Y:${Math.round(y*100)}cm, T:${Math.round(theta*57.3)}deg`);
        
        for (let row = 0; row < MAP_DIM; row++) {
            let line = "";
            for (let col = 0; col < MAP_DIM; col++) {
                let cellData = mapGrid[row * MAP_DIM + col];
                
                // Draw robot location as 'O'
                let robotGridX = Math.floor(x / GRID_RES) + (MAP_DIM / 2);
                let robotGridY = Math.floor(y / GRID_RES) + (MAP_DIM / 2);
                
                if (col === robotGridX && row === robotGridY) {
                    line += "O "; // Robot is here
                } 
                else if (cellData > 100) line += "██"; // Solid Wall
                else if (cellData > 25)  line += "▒▒";  // Unsure / Ghost
                else line += "· ";                     // Empty Space
            }
            serial.writeLine(line);
        }
        serial.writeLine("-----------------------------");
    }
}
