/**
 * Chapter 2: Linear Combinations & Span
 * 
 * IMPORTANT: This file follows the EXACT same pattern as e01-vectors.js
 * which is confirmed working. Key patterns:
 * 1. Use baseDraw = canvas.draw.bind(canvas) to save original draw
 * 2. Override canvas.draw and call baseDraw() first
 * 3. Add custom rendering AFTER baseDraw()
 * 4. Use this.toCanvas(0, 0) for origin, NOT this.origin
 */
window.chapterScripts['e02-combinations'] = function () {
    console.log('Initializing Chapter 2 (Linear Combinations)');

    // === INTERACTIVE 1: Basis Change (Twisted Grid) ===
    (function initBasisChange() {
        const canvasEl = document.getElementById('viz-basis-change');
        if (!canvasEl) {
            console.error('Canvas viz-basis-change not found');
            return;
        }

        const canvas = new CanvasSpace('viz-basis-change', { interactive: true });

        // Basis vectors (mutable)
        let i_hat = new Vector2D(1, 0);
        let j_hat = new Vector2D(0, 1);

        // Linear combination coefficients for the result vector
        const c1 = 2;
        const c2 = 1;

        // Add basis vectors to canvas
        const iObj = canvas.addVector(i_hat, '#f97316', 'î');
        const jObj = canvas.addVector(j_hat, '#10b981', 'ĵ');

        // Make them draggable
        canvas.makeDraggable(iObj, (v) => {
            i_hat = v;
            canvas.draw();
            updateBasisDisplay();
        });
        canvas.makeDraggable(jObj, (v) => {
            j_hat = v;
            canvas.draw();
            updateBasisDisplay();
        });

        // Save original draw method
        const baseDraw = canvas.draw.bind(canvas);

        // Override draw to add custom grid and result vector
        canvas.draw = function () {
            // 1. Clear and draw standard axes (not the standard grid, we'll draw our own)
            this.clear();

            const ctx = this.ctx;
            const origin = this.toCanvas(0, 0);

            // 2. Draw custom skewed grid based on current basis
            ctx.save();
            ctx.lineWidth = 1;
            ctx.strokeStyle = 'rgba(74, 158, 255, 0.15)';

            const range = 15;

            // Lines parallel to j_hat (stepping along i_hat direction)
            for (let k = -range; k <= range; k++) {
                const startX = k * i_hat.x - range * j_hat.x;
                const startY = k * i_hat.y - range * j_hat.y;
                const endX = k * i_hat.x + range * j_hat.x;
                const endY = k * i_hat.y + range * j_hat.y;

                const s = this.toCanvas(startX, startY);
                const e = this.toCanvas(endX, endY);

                ctx.beginPath();
                ctx.moveTo(s.x, s.y);
                ctx.lineTo(e.x, e.y);
                ctx.stroke();
            }

            // Lines parallel to i_hat (stepping along j_hat direction)
            for (let k = -range; k <= range; k++) {
                const startX = -range * i_hat.x + k * j_hat.x;
                const startY = -range * i_hat.y + k * j_hat.y;
                const endX = range * i_hat.x + k * j_hat.x;
                const endY = range * i_hat.y + k * j_hat.y;

                const s = this.toCanvas(startX, startY);
                const e = this.toCanvas(endX, endY);

                ctx.beginPath();
                ctx.moveTo(s.x, s.y);
                ctx.lineTo(e.x, e.y);
                ctx.stroke();
            }
            ctx.restore();

            // 3. Draw the result vector v = c1*i + c2*j (Yellow)
            const vx = c1 * i_hat.x + c2 * j_hat.x;
            const vy = c1 * i_hat.y + c2 * j_hat.y;
            const vEnd = this.toCanvas(vx, vy);
            this.drawArrow(origin, vEnd, '#fbbf24', 'v');

            // 4. Draw the basis vectors (on top)
            const iEnd = this.toCanvas(i_hat.x, i_hat.y);
            const jEnd = this.toCanvas(j_hat.x, j_hat.y);
            this.drawArrow(origin, iEnd, '#f97316', 'î');
            this.drawArrow(origin, jEnd, '#10b981', 'ĵ');
        };

        // Reset button
        const btnReset = document.getElementById('reset-basis');
        if (btnReset) {
            btnReset.addEventListener('click', () => {
                // Reset to standard basis
                i_hat.x = 1; i_hat.y = 0;
                j_hat.x = 0; j_hat.y = 1;
                // Also update the vector objects
                iObj.v.x = 1; iObj.v.y = 0;
                jObj.v.x = 0; jObj.v.y = 1;
                canvas.draw();
                updateBasisDisplay();
            });
        }

        function updateBasisDisplay() {
            const displayEl = document.getElementById('basis-numeric-display');
            if (displayEl && window.katex) {
                // Current basis vectors
                const ix = i_hat.x.toFixed(1);
                const iy = i_hat.y.toFixed(1);
                const jx = j_hat.x.toFixed(1);
                const jy = j_hat.y.toFixed(1);

                // Result vector
                const vx = (c1 * i_hat.x + c2 * j_hat.x).toFixed(1);
                const vy = (c1 * i_hat.y + c2 * j_hat.y).toFixed(1);

                katex.render(
                    `2\\color{#f97316}{\\begin{bmatrix}${ix}\\\\${iy}\\end{bmatrix}} + 1\\color{#10b981}{\\begin{bmatrix}${jx}\\\\${jy}\\end{bmatrix}} = \\color{#fbbf24}{\\begin{bmatrix}${vx}\\\\${vy}\\end{bmatrix}}`,
                    displayEl,
                    { throwOnError: false, displayMode: true }
                );
            }
        }

        // Initial draw
        canvas.draw();
        updateBasisDisplay();
    })();


    // === INTERACTIVE 2: Span Visualizer ===
    (function initSpanCheck() {
        const canvasEl = document.getElementById('viz-span-check');
        if (!canvasEl) {
            console.error('Canvas viz-span-check not found');
            return;
        }

        const canvas = new CanvasSpace('viz-span-check', { interactive: true });

        // Two vectors to check for linear independence
        let v1 = new Vector2D(2, 1);
        let v2 = new Vector2D(-1, 2);

        // Add to canvas
        const v1Obj = canvas.addVector(v1, '#4a9eff', 'v₁');
        const v2Obj = canvas.addVector(v2, '#f97316', 'v₂');

        const statusEl = document.getElementById('span-status');

        // Function to check if vectors are linearly dependent
        function updateSpanStatus() {
            // Calculate determinant (2D "cross product")
            const det = v1.x * v2.y - v1.y * v2.x;
            const isCollinear = Math.abs(det) < 0.3;

            if (statusEl) {
                if (isCollinear) {
                    statusEl.textContent = '直线 (1D) - 线性相关!';
                    statusEl.style.color = '#ef4444'; // Red
                } else {
                    statusEl.textContent = '平面 (2D) - 线性无关';
                    statusEl.style.color = '#4a9eff'; // Blue
                }
            }
        }

        // Make draggable with update callback
        canvas.makeDraggable(v1Obj, (v) => {
            v1 = v;
            updateSpanStatus();
            canvas.draw();
        });
        canvas.makeDraggable(v2Obj, (v) => {
            v2 = v;
            updateSpanStatus();
            canvas.draw();
        });

        // Save original draw
        const baseDraw = canvas.draw.bind(canvas);

        // Override draw to add span visualization
        canvas.draw = function () {
            // 1. Call base draw (clears, draws grid, draws vectors)
            baseDraw();

            const ctx = this.ctx;
            const det = v1.x * v2.y - v1.y * v2.x;
            const isCollinear = Math.abs(det) < 0.3;

            if (isCollinear && (v1.x !== 0 || v1.y !== 0)) {
                // Draw the line they span
                ctx.save();
                ctx.strokeStyle = 'rgba(239, 68, 68, 0.4)';
                ctx.lineWidth = 8;

                // Direction from whichever vector is non-zero
                let dir = v1;
                if (Math.abs(v1.x) < 0.01 && Math.abs(v1.y) < 0.01) dir = v2;

                // Draw infinite line through origin
                const farPos = this.toCanvas(dir.x * 50, dir.y * 50);
                const farNeg = this.toCanvas(-dir.x * 50, -dir.y * 50);

                ctx.beginPath();
                ctx.moveTo(farNeg.x, farNeg.y);
                ctx.lineTo(farPos.x, farPos.y);
                ctx.stroke();
                ctx.restore();
            }
        };

        // Initial state
        updateSpanStatus();
        canvas.draw();
    })();
};
