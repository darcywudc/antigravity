/**
 * Chapter 3: Linear Transformations
 */
window.chapterScripts['e03-transformations'] = function () {
    console.log('Initializing Chapter 3 (Essence)');

    const canvas = new CanvasSpace('viz-matrix-transform', {
        interactive: true,
        bg: '#0d0d12'
    });

    // Current transformation matrix values
    let a = 1, b = 0, c = 0, d = 1;

    // Inputs
    const inputA = document.getElementById('val-a');
    const inputB = document.getElementById('val-b');
    const inputC = document.getElementById('val-c');
    const inputD = document.getElementById('val-d');

    function updateMatrix(newA, newB, newC, newD, updateInputs = false) {
        a = Number(newA); b = Number(newB);
        c = Number(newC); d = Number(newD);

        if (updateInputs) {
            inputA.value = a; inputB.value = b;
            inputC.value = c; inputD.value = d;
        }
        canvas.draw();
    }

    [inputA, inputB, inputC, inputD].forEach(input => {
        input.addEventListener('input', () => {
            updateMatrix(inputA.value, inputB.value, inputC.value, inputD.value);
        });
    });

    // Preset Buttons
    document.getElementById('btn-shear').addEventListener('click', () => {
        // Shear i-hat stays, j-hat moves to (1,1)
        animateMatrix(1, 1, 0, 1);
    });

    document.getElementById('btn-rotate').addEventListener('click', () => {
        // Rotate 45 degrees
        // i -> (cos45, sin45), j -> (-sin45, cos45)
        // 0.707
        animateMatrix(0.707, -0.707, 0.707, 0.707);
    });

    document.getElementById('btn-scale').addEventListener('click', () => {
        // Simple scaling
        animateMatrix(2, 0, 0, 2);
    });

    // Animation helper
    function animateMatrix(targetA, targetB, targetC, targetD) {
        // Simple linear interpolation
        const steps = 30;
        const startA = a, startB = b, startC = c, startD = d;
        let frame = 0;

        function loop() {
            frame++;
            const t = frame / steps;
            // Ease out
            const ease = t * (2 - t);

            updateMatrix(
                startA + (targetA - startA) * ease,
                startB + (targetB - startB) * ease,
                startC + (targetC - startC) * ease,
                startD + (targetD - startD) * ease,
                true // update inputs during animation
            );

            if (frame < steps) requestAnimationFrame(loop);
        }
        loop();
    }


    canvas.draw = function () {
        this.clear();
        const ctx = this.ctx;

        // Custom Grid Transformation Logic
        // Transform function
        const transform = (x, y) => {
            // Apply matrix [a b; c d]
            const tx = a * x + b * y;
            const ty = c * x + d * y;
            return this.toCanvas(tx, ty);
        };

        // Draw Transformed Grid
        ctx.lineWidth = 1;
        ctx.strokeStyle = 'rgba(74, 158, 255, 0.2)';

        // Vertical lines (initially vertical, now transformed)
        // They represent x = constant lines. Moving along y changes the point.
        // We iterate x from -10 to 10
        for (let i = -15; i <= 15; i++) {
            ctx.beginPath();
            // Line corresponding to x = i. y goes from min to max.
            const start = transform(i, -15);
            const end = transform(i, 15);
            ctx.moveTo(start.x, start.y);
            ctx.lineTo(end.x, end.y);
            ctx.stroke();
        }

        // Horizontal lines (initially horizontal)
        for (let j = -15; j <= 15; j++) {
            ctx.beginPath();
            const start = transform(-15, j);
            const end = transform(15, j);
            ctx.moveTo(start.x, start.y);
            ctx.lineTo(end.x, end.y);
            ctx.stroke();
        }

        // Draw Basis Vectors
        const origin = this.toCanvas(0, 0);

        // Transformed i-hat: (1, 0) -> (a, c)
        const i_hat_trans = transform(1, 0);
        this.drawArrow(origin, i_hat_trans, '#f97316', 'i', 3);

        // Transformed j-hat: (0, 1) -> (b, d)
        const j_hat_trans = transform(0, 1); // logic error here? matrix is [a b; c d]
        // [a b] * [0] = [b]
        // [c d]   [1]   [d]
        // Yes correct.
        this.drawArrow(origin, j_hat_trans, '#10b981', 'j', 3);
    };

    canvas.draw();
};
