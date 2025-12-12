/**
 * Chapter 4: Matrix Multiplication
 * Visualizing Composition
 */
window.chapterScripts['e04-multiplication'] = function () {
    console.log('Initializing Chapter 4 (Essence)');

    const canvas = new CanvasSpace('viz-composition', {
        interactive: false, // Animation driven
        bg: '#0d0d12'
    });

    // We will simulate 2 matrices
    // M1: Rotate 90 deg (Counter-clockwise) = [0 -1; 1 0]
    // M2: Shear right = [1 1; 0 1]

    // Note in text I used Rotate then Shear.
    // Let's stick to the button labels.
    // "Rotate then Shear" means Shear(Rotate(v)) => Matrix is M_shear * M_rotate

    // Matrix definitions
    const M_rotate = new Matrix(2, 2, [[0, -1], [1, 0]]); // 90 deg
    const M_shear = new Matrix(2, 2, [[1, 1], [0, 1]]);   // Shear x

    // We need to animate the grid state
    // State is simply the current transformation matrix applied to the grid
    let currentMatrix = Matrix.identity(2);
    let animationQueue = [];
    let isAnimating = false;

    // Helper to linear interpolate matrices
    function lerpMatrix(m1, m2, t) {
        const res = new Matrix(2, 2);
        for (let r = 0; r < 2; r++) {
            for (let c = 0; c < 2; c++) {
                const val = m1.get(r, c) + (m2.get(r, c) - m1.get(r, c)) * t;
                res.set(r, c, val);
            }
        }
        return res;
    }

    function processQueue() {
        if (isAnimating || animationQueue.length === 0) return;

        isAnimating = true;
        const targetConfig = animationQueue.shift();
        const startMatrix = currentMatrix.clone();
        const targetMatrix = targetConfig.matrix;

        let frame = 0;
        const totalFrames = 60;

        function loop() {
            frame++;
            const t = frame / totalFrames;
            const ease = t < .5 ? 2 * t * t : -1 + (4 - 2 * t) * t; // Ease in out

            currentMatrix = lerpMatrix(startMatrix, targetMatrix, ease);
            canvas.draw();

            if (frame < totalFrames) {
                requestAnimationFrame(loop);
            } else {
                isAnimating = false;
                processQueue(); // Next step
            }
        }
        loop();
    }

    // Button: Step by Step
    // 1. Rotate
    // 2. Then Shear
    // Result = M_shear * M_rotate
    document.getElementById('btn-step-by-step').addEventListener('click', () => {
        // Reset first
        currentMatrix = Matrix.identity(2);
        animationQueue = [];

        // Step 1: Rotate input IDENTITY
        // Target is M_rotate
        animationQueue.push({ matrix: M_rotate });

        // Step 2: Apply Shear TO the current state
        // If current state is M_rot, new state is M_shear * M_rot
        const finalMat = M_shear.multiply(M_rotate); // Order: Shear(Rotate)
        animationQueue.push({ matrix: finalMat });

        processQueue();
    });

    // Button: Composite
    // Go straight to M_shear * M_rotate
    document.getElementById('btn-composite').addEventListener('click', () => {
        currentMatrix = Matrix.identity(2);
        animationQueue = [];

        const finalMat = M_shear.multiply(M_rotate);
        animationQueue.push({ matrix: finalMat });

        processQueue();
    });

    document.getElementById('btn-reset-comp').addEventListener('click', () => {
        animationQueue = [];
        animationQueue.push({ matrix: Matrix.identity(2) });
        processQueue();
    });

    canvas.draw = function () {
        this.clear();
        const ctx = this.ctx;

        // Use the currentMatrix to store 'a,b,c,d'
        const a = currentMatrix.get(0, 0);
        const b = currentMatrix.get(0, 1);
        const c = currentMatrix.get(1, 0);
        const d = currentMatrix.get(1, 1);

        const transform = (x, y) => {
            const tx = a * x + b * y;
            const ty = c * x + d * y;
            return this.toCanvas(tx, ty);
        };

        // Draw Grid
        ctx.lineWidth = 1;
        ctx.strokeStyle = 'rgba(255, 255, 255, 0.1)';

        for (let i = -10; i <= 10; i++) {
            ctx.beginPath();
            const start = transform(i, -10);
            const end = transform(i, 10);
            ctx.moveTo(start.x, start.y);
            ctx.lineTo(end.x, end.y);
            ctx.stroke();

            ctx.beginPath();
            const startH = transform(-10, i);
            const endH = transform(10, i);
            ctx.moveTo(startH.x, startH.y);
            ctx.lineTo(endH.x, endH.y);
            ctx.stroke();
        }

        // Draw Basis
        const origin = this.toCanvas(0, 0);
        const i_hat = transform(1, 0);
        const j_hat = transform(0, 1);

        this.drawArrow(origin, i_hat, '#f97316', 'i', 3);
        this.drawArrow(origin, j_hat, '#10b981', 'j', 3);

        // Display Matrix Values on screen?
        // Maybe later.
    };

    canvas.draw();
};
