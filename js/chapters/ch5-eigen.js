/**
 * Chapter 5 Scripts: Eigenvalues and Eigenvectors
 */
window.chapterScripts['ch5-eigen'] = function () {
    window.logDebug && window.logDebug('Initializing Chapter 5');

    const canvas = new CanvasSpace('viz-eigen', { interactive: true, scale: 50 });

    // Default matrix: Horizontal Stretch (Scale X by 2)
    // Eigenvectors: (1,0) with lambda=2, and (0,1) with lambda=1
    let matrix = new Matrix(2, 2, [[2, 0], [0, 1]]);

    // The interactive vector x (normalized initially)
    let vecX = new Vector2D(1, 0).rotate(Math.PI / 6); // Start at 30 deg

    // Add draggable tip for vector x
    // To implement "drag around circle", we need custom logic.
    // CanvasSpace's simple drag logic might modify length.
    // We want to normalize it after drag.

    const xObj = canvas.addVector(vecX, '#4a9eff', 'x');

    canvas.makeDraggable(xObj, (newV) => {
        // Constrain to unit circle roughly, or just let users drag length
        // But for eigenvector viz, direction is what matters.
        // Let's keep length flexible but normalize for calculation check.

        // Actually, let's just use the current v and calculate Ax
        // But to make it nicer, maybe we normalize the INPUT vector x for the checking logic
        // but let the visual vector be whatever.
        // Or better: Normalize it upon release? Or just keep it as is.
        // Let's keep as is.

        updateEigenCheck();
    });

    // Result vector Ax (not draggable)
    let Ax = new Vector2D(0, 0);
    canvas.addVector(Ax, '#fbbf24', 'Ax');

    const updateEigenCheck = () => {
        // Calculate Ax
        const res = matrix.multiplyVector([vecX.x, vecX.y]);
        Ax.x = res[0];
        Ax.y = res[1];

        // Check collinearity
        // Cross product 2D value: x1*y2 - x2*y1
        const cross = vecX.x * Ax.y - vecX.y * Ax.x;
        // Tolerance relative to magnitudes
        const tolerance = 0.1 * vecX.magnitude() * Ax.magnitude(); // Loose tolerance for interaction

        const isAligned = Math.abs(cross) < 0.5; // Simple threshold

        const statusEl = document.getElementById('eigen-status');

        if (isAligned && vecX.magnitude() > 0.1) {
            // Estimate Lambda
            // Ax = lambda * x => lambda = |Ax| / |x| * sign
            let lambda = Ax.magnitude() / vecX.magnitude();
            // Check direction (dot product)
            if (vecX.dot(Ax) < 0) lambda = -lambda;

            statusEl.innerHTML = `$$ A\\mathbf{x} \\approx ${lambda.toFixed(2)}\\mathbf{x} $$ <span style="color:#10b981">(Found!)</span>`;
            canvas.canvas.style.backgroundColor = '#101a15'; // Subtle hint
        } else {
            statusEl.innerHTML = `$$ A\\mathbf{x} \\neq \\lambda\\mathbf{x} $$`;
            canvas.canvas.style.backgroundColor = '#0d0d12';
        }

        canvas.draw();
    };

    // Buttons
    const btn1 = document.getElementById('btn-eigen-1');
    const btn2 = document.getElementById('btn-eigen-2');
    const btn3 = document.getElementById('btn-eigen-3');

    const setActive = (btn) => {
        [btn1, btn2, btn3].forEach(b => b.className = 'btn btn-secondary');
        btn.className = 'btn btn-primary';
        updateEigenCheck();
    };

    btn1.addEventListener('click', () => {
        matrix = new Matrix(2, 2, [[2, 0], [0, 1]]);
        setActive(btn1);
    });

    btn2.addEventListener('click', () => {
        matrix = new Matrix(2, 2, [[1, 1], [0, 1]]); // Shear
        setActive(btn2);
    });

    btn3.addEventListener('click', () => {
        matrix = new Matrix(2, 2, [[2, 1], [1, 2]]); // Symmetric
        setActive(btn3);
    });

    // Initial update
    updateEigenCheck();
};
