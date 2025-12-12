/**
 * Chapter 2: Linear Combinations & Span (Enhanced)
 */
window.chapterScripts['e02-combinations'] = function () {
    console.log('Initializing Chapter 2 (Enhanced Essence)');

    // --- 1. Changing Basis Interactive (The Twisted Grid) ---
    (function initBasisChange() {
        const canvas = new CanvasSpace('viz-basis-change', {
            interactive: true,
            gridColor: 'rgba(255,255,255,0.05)'
        });

        // Initial Basis
        let i_hat = new Vector2D(1, 0);
        let j_hat = new Vector2D(0, 1);

        // Coefficients fixed: v = 2i + 1j
        const c1 = 2;
        const c2 = 1;

        // Custom draw for skewed grid
        canvas.draw = function () {
            this.clear();
            const ctx = this.ctx;

            // Draw skewed grid lines
            ctx.save();
            ctx.lineWidth = 1;
            ctx.strokeStyle = 'rgba(74, 158, 255, 0.15)';

            const range = 15;

            // Lines parallel to j (stepping along i)
            for (let k = -range; k <= range; k++) {
                const s = this.toCanvas(
                    k * i_hat.x - range * j_hat.x,
                    k * i_hat.y - range * j_hat.y
                );
                const e = this.toCanvas(
                    k * i_hat.x + range * j_hat.x,
                    k * i_hat.y + range * j_hat.y
                );
                ctx.beginPath(); ctx.moveTo(s.x, s.y); ctx.lineTo(e.x, e.y); ctx.stroke();
            }
            // Lines parallel to i (stepping along j)
            for (let k = -range; k <= range; k++) {
                const s = this.toCanvas(
                    -range * i_hat.x + k * j_hat.x,
                    -range * i_hat.y + k * j_hat.y
                );
                const e = this.toCanvas(
                    range * i_hat.x + k * j_hat.x,
                    range * i_hat.y + k * j_hat.y
                );
                ctx.beginPath(); ctx.moveTo(s.x, s.y); ctx.lineTo(e.x, e.y); ctx.stroke();
            }
            ctx.restore();

            // Draw resulting vector v = 2i + 1j
            const vx = c1 * i_hat.x + c2 * j_hat.x;
            const vy = c1 * i_hat.y + c2 * j_hat.y;
            // Draw it yellow and thicker
            this.drawArrow(this.origin, this.toCanvas(vx, vy), '#fbbf24', 'v', 3);

            // We rely on base draw for i and j vectors? 
            // Actually base draw() loops through this.vectors and draws them.
            // So we just need to Add them effectively.
            // But we want i and j on TOP. The iteration order matters.

            // Manually draw draggables on top if we want? 
            // CanvasSpace.draw() draws grid then vectors. 
            // So calling base draw here (this.draw is overridden) is tricky unless we manually reproduce logic.

            // Let's manually render the basis vectors here to ensure they are on top of our custom grid
            // But wait, the standard loop is usually fine.
            // Let's iterate through registered vectors and draw them.
            for (let item of this.vectors) {
                const start = this.toCanvas(0, 0); // Assume origin based
                const end = this.toCanvas(item.v.x, item.v.y);
                this.drawArrow(start, end, item.color, item.label);
            }
        };

        // Add draggable basis vectors
        const iObj = canvas.addVector(i_hat, '#f97316', 'i');
        canvas.makeDraggable(iObj, (v) => { i_hat = v; });

        const jObj = canvas.addVector(j_hat, '#10b981', 'j');
        canvas.makeDraggable(jObj, (v) => { j_hat = v; });

        // Reset
        const btnReset = document.getElementById('reset-basis');
        if (btnReset) {
            btnReset.onclick = () => {
                i_hat.x = 1; i_hat.y = 0;
                j_hat.x = 0; j_hat.y = 1;
                canvas.draw();
            };
        }

        canvas.draw();
    })();


    // --- 2. Span & Independence Visualizer (New) ---
    (function initSpanCheck() {
        const canvas = new CanvasSpace('viz-span-check', { interactive: true });

        // Two vectors
        let v1 = new Vector2D(2, 1);
        let v2 = new Vector2D(-1, 2); // Independent initially

        const v1Obj = canvas.addVector(v1, '#4a9eff', 'v1');
        const v2Obj = canvas.addVector(v2, '#f97316', 'v2');

        const statusEl = document.getElementById('span-status');

        function checkSpan() {
            // Check determinant (2D cross product)
            const det = v1.x * v2.y - v1.y * v2.x;
            const isLine = Math.abs(det) < 0.2; // Threshold for UX ease

            // Update Text
            if (statusEl) {
                if (isLine) {
                    statusEl.textContent = "直线 (1D) - 线性相关!";
                    statusEl.style.color = "var(--accent-red)";
                } else {
                    statusEl.textContent = "平面 (2D) - 线性无关";
                    statusEl.style.color = "var(--accent-blue)";
                }
            }

            // Update Canvas Visuals
            canvas.draw = function () {
                this.clear();

                const ctx = this.ctx;

                if (isLine) {
                    // Draw the infinite line they are stuck on
                    // Line passes through origin and v1 (since v1 ~= v2 direction)
                    // If v1 is zero, try v2. If both zero, just point.
                    let dir = v1;
                    if (v1.length() < 0.1) dir = v2;

                    if (dir.length() > 0.1) {
                        // Draw line
                        ctx.save();
                        ctx.strokeStyle = 'rgba(239, 68, 68, 0.5)'; // Red tint
                        ctx.lineWidth = 4;
                        const farPos = this.toCanvas(dir.x * 100, dir.y * 100);
                        const farNeg = this.toCanvas(dir.x * -100, dir.y * -100);
                        ctx.beginPath();
                        ctx.moveTo(farNeg.x, farNeg.y);
                        ctx.lineTo(farPos.x, farPos.y);
                        ctx.stroke();
                        ctx.restore();
                    }
                } else {
                    // Draw Plane Visual (Faint Blue Field)
                    this.clear(); // Black bg
                    this.ctx.fillStyle = 'rgba(74, 158, 255, 0.05)';
                    this.ctx.fillRect(0, 0, this.width, this.height);

                    this.drawGrid(); // Standard grid on top
                }

                // Draw vectors
                for (let item of this.vectors) {
                    const start = this.toCanvas(0, 0);
                    const end = this.toCanvas(item.v.x, item.v.y);
                    this.drawArrow(start, end, item.color, item.label);
                }
            };

            canvas.draw();
        }

        canvas.makeDraggable(v1Obj, (v) => { v1 = v; checkSpan(); });
        canvas.makeDraggable(v2Obj, (v) => { v2 = v; checkSpan(); });

        checkSpan(); // Initial check
    })();
};
