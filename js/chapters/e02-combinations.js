/**
 * Chapter 2: Linear Combinations & Span
 */
window.chapterScripts['e02-combinations'] = function () {
    console.log('Initializing Chapter 2 (Essence)');

    const canvas = new CanvasSpace('viz-basis-change', {
        interactive: true,
        gridColor: 'rgba(255,255,255,0.05)'
    });

    // Initial Basis
    let i_hat = new Vector2D(1, 0);
    let j_hat = new Vector2D(0, 1);

    // The coefficients are fixed for the demonstration: v = 2*i + 1*j
    const c1 = 2;
    const c2 = 1;

    // Custom draw function to show the skewed grid
    canvas.draw = function () {
        this.clear();

        const ctx = this.ctx;

        // 1. Draw the skewed grid based on current basis
        // We draw lines parallel to i_hat and j_hat
        ctx.save();
        ctx.lineWidth = 1;
        ctx.strokeStyle = 'rgba(74, 158, 255, 0.15)'; // Faint blue grid

        const range = 15; // How many lines to draw

        // Lines parallel to j_hat (stepping along i_hat)
        for (let k = -range; k <= range; k++) {
            // Start point: k * i_hat - range * j_hat
            // End point:   k * i_hat + range * j_hat

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

        // Lines parallel to i_hat (stepping along j_hat)
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

        // 2. Draw axes (standard x/y axes for reference? No, maybe just origin)
        // this.drawAxes(); // Let's skip standard axes to emphasize the custom basis grid

        // 3. Draw the resulting vector v = 2i + 1j
        const vx = c1 * i_hat.x + c2 * j_hat.x;
        const vy = c1 * i_hat.y + c2 * j_hat.y;
        this.drawArrow(this.origin, this.toCanvas(vx, vy), '#fbbf24', 'v', 2); // Yellow

        // 4. Draw the basis vectors LAST so they are on top & draggable handles work
        // Note: CanvasSpace doesn't support custom drag logic for arbitrary objects easily unless we register them.
        // We manually added them below. The draw loop handles rendering.
    };

    // Add draggable basis vectors
    const iObj = canvas.addVector(i_hat, '#f97316', 'i');
    canvas.makeDraggable(iObj, (v) => {
        i_hat = v;
    });

    const jObj = canvas.addVector(j_hat, '#10b981', 'j');
    canvas.makeDraggable(jObj, (v) => {
        j_hat = v;
    });

    // Reset button
    const btnReset = document.getElementById('reset-basis');
    if (btnReset) {
        btnReset.addEventListener('click', () => {
            i_hat.x = 1; i_hat.y = 0;
            j_hat.x = 0; j_hat.y = 1;
            canvas.draw();
        });
    }

    canvas.draw();
};
