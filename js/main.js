// Savitzky-Golay Filter implementation
class SavitzkyGolayFilter {
    constructor(windowSize = 5, polynomialOrder = 2) {
        this.windowSize = windowSize;
        this.polynomialOrder = polynomialOrder;
        this.buffer = [];
        // Precomputed coefficients for windowSize=5, polynomialOrder=2
        this.coefficients = [3, -5, -3, 9, 31];
        this.denominator = 35;
    }

    smooth(newValue) {
        this.buffer.push(newValue);
        if (this.buffer.length > this.windowSize) {
            this.buffer.shift();
        }
        if (this.buffer.length < this.windowSize) {
            return newValue; // Return raw value until buffer is filled
        }
        let smoothed = 0;
        for (let i = 0; i < this.windowSize; i++) {
            smoothed += this.buffer[i] * this.coefficients[i];
        }
        return smoothed / this.denominator;
    }
}

// Kalman Filter implementation
class KalmanFilter {
    constructor({ R = 1, Q = 1, A = 1, B = 0, C = 1 } = {}) {
        this.R = R;
        this.Q = Q;
        this.A = A;
        this.B = B;
        this.C = C;
        this.cov = NaN;
        this.x = NaN;
    }
    filter(z, u = 0) {
        if (isNaN(this.x)) {
            this.x = (1 / this.C) * z;
            this.cov = (1 / this.C) * this.Q * (1 / this.C);
        } else {
            const predX = (this.A * this.x) + (this.B * u);
            const predCov = ((this.A * this.cov) * this.A) + this.R;
            const K = predCov * this.C * (1 / ((this.C * predCov * this.C) + this.Q));
            this.x = predX + K * (z - (this.C * predX));
            this.cov = predCov - (K * this.C * predCov);
        }
        return this.x;
    }
}

const THRESHOLD_DISTANCE = 50;
const kalmanFilterX = new KalmanFilter();
const kalmanFilterY = new KalmanFilter();
const sgFilterX = new SavitzkyGolayFilter();
const sgFilterY = new SavitzkyGolayFilter();

window.onload = async function() {
    await webgazer.setRegression('ridge')
        .setTracker('clmtrackr')
        .setGazeListener(function(data, clock) {
            if (data && data.x !== null && data.y !== null) {
                const filteredData = filterGazePoint(data.x, data.y);
                if (filteredData) {
                    const kalmanX = kalmanFilterX.filter(filteredData.x);
                    const kalmanY = kalmanFilterY.filter(filteredData.y);
                    const smoothedX = sgFilterX.smooth(kalmanX);
                    const smoothedY = sgFilterY.smooth(kalmanY);
                    console.log({ x: smoothedX, y: smoothedY });
                    updateScatterPlot([smoothedX], [smoothedY]);
                }
            }
        })
        .saveDataAcrossSessions(true)
        .begin();

    webgazer.showVideoPreview(true)
        .showPredictionPoints(true)
        .applyKalmanFilter(true);

    var setup = function() {
        var canvas = document.getElementById("plotting_canvas");
        canvas.width = window.innerWidth;
        canvas.height = window.innerHeight;
        canvas.style.position = 'fixed';
    };
    setup();
};

// Rest of the code remains the same
window.saveDataAcrossSessions = true;
window.onbeforeunload = function() { webgazer.end(); }

function Restart() {
    document.getElementById("Accuracy").innerHTML = "<a>Not yet Calibrated</a>";
    webgazer.clearData();
    ClearCalibration();
    PopUpInstruction();
}

function filterGazePoint(x, y) {
    const centerX = window.innerWidth / 2;
    const centerY = window.innerHeight / 2;
    const distance = Math.sqrt(Math.pow(x - centerX, 2) + Math.pow(y - centerY, 2));
    return distance < THRESHOLD_DISTANCE ? { x, y } : null;
}

function updateScatterPlot(x, y) {
    Plotly.newPlot('plotting_canvas', [{
        x: x,
        y: y,
        mode: 'markers',
        type: 'scatter'
    }], {
        title: 'Live Gaze Position Scatter Plot',
        xaxis: { title: 'X Position' },
        yaxis: { title: 'Y Position' }
    });
}