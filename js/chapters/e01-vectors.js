/**
 * Chapter 1: Vectors (Enhanced)
 * Supports multiple interactive visualizations
 */
window.chapterScripts['e01-vectors'] = function () {
    console.log('Initializing Chapter 1 (Enhanced Essence)');

    // --- 1. Basic Coordinate System Interactive ---
    (function initBasic() {
        const canvas = new CanvasSpace('viz-vector-basic', { interactive: true });
        const vector = new Vector2D(2, 1);

        const vecObj = canvas.addVector(vector, '#4a9eff', 'v');
        canvas.makeDraggable(vecObj, (v) => {
            updateDisplay(v);
        });

        function updateDisplay(v) {
            const display = document.getElementById('vector-display');
            if (display && window.katex) {
                const x = parseFloat(v.x).toFixed(1); // Keep 1 decimal
                const y = parseFloat(v.y).toFixed(1);
                katex.render(
                    `\\vec{v} = \\begin{bmatrix} ${x} \\\\ ${y} \\end{bmatrix}`,
                    display,
                    { throwOnError: false, displayMode: true }
                );
            }
        }

        canvas.draw();
        updateDisplay(vector);
    })();


    // --- 2. Vector Addition Interactive ---
    (function initAddition() {
        const canvas = new CanvasSpace('viz-vector-add', { interactive: true });

        // Two source vectors
        let v = new Vector2D(1, 2);
        let w = new Vector2D(3, -1);

        // Add vectors
        const vObj = canvas.addVector(v, '#4a9eff', 'v');
        const wObj = canvas.addVector(w, '#f97316', 'w');

        // Make draggable
        canvas.makeDraggable(vObj, (newV) => {
            v = newV; // update ref
            canvas.draw(); // redraw sum
        });
        canvas.makeDraggable(wObj, (newW) => {
            w = newW;
            canvas.draw();
        });

        // Custom draw to show the "Head-to-Tail" or "Sum" vector
        // We override canvas.draw but call the base drawing logic inside or re-implement
        // Actually CanvasSpace.draw draws all `this.vectors`. 
        // We want to dynamically draw the Sum vector without making it a permanent "draggable" object in the list
        // Or we can just add a 3rd vector to the list but not make it draggable.

        // Let's hook into post-draw? CanvasSpace doesn't have hooks yet.
        // We can override draw.
        const baseDraw = canvas.draw.bind(canvas);

        canvas.draw = function () {
            // 1. Draw standard grid and v, w
            baseDraw();

            const ctx = this.ctx;

            // 2. Draw Sum vector (Green)
            const sum = new Vector2D(v.x + w.x, v.y + w.y);
            const origin = this.toCanvas(0, 0);
            const sumEnd = this.toCanvas(sum.x, sum.y);

            this.drawArrow(origin, sumEnd, '#10b981', 'v+w');

            // 3. Draw "Ghost" guidelines to show movement
            // Ghost of w starting at v
            const vEnd = this.toCanvas(v.x, v.y);
            // w ends at sumEnd

            ctx.beginPath();
            ctx.setLineDash([5, 5]);
            ctx.lineWidth = 1;
            ctx.strokeStyle = 'rgba(249, 115, 22, 0.5)'; // Orange ghost (w)
            ctx.moveTo(vEnd.x, vEnd.y);
            ctx.lineTo(sumEnd.x, sumEnd.y);
            ctx.stroke();

            // Ghost of v starting at w (Parallelogram)
            const wEnd = this.toCanvas(w.x, w.y);
            ctx.beginPath();
            ctx.strokeStyle = 'rgba(74, 158, 255, 0.5)'; // Blue ghost (v)
            ctx.moveTo(wEnd.x, wEnd.y);
            ctx.lineTo(sumEnd.x, sumEnd.y);
            ctx.stroke();

            ctx.setLineDash([]);
        };

        canvas.draw();
    })();


    // --- 3. Scalar Multiplication Interactive ---
    (function initScaling() {
        const canvas = new CanvasSpace('viz-vector-scale', { interactive: true });

        // Base vector
        let v = new Vector2D(2, 1);
        let scalar = 2.0;

        // Visual scalar vector
        // We want to show the ORIGINAL v (ghost) and Scaled V (solid)

        // Let's make v draggable to set direction
        const vObj = canvas.addVector(v, 'rgba(255,255,255,0.2)', 'v_orig'); // faint original
        canvas.makeDraggable(vObj, (newV) => {
            v = newV;
            canvas.draw();
        });

        const slider = document.getElementById('scalar-slider');
        const valDisplay = document.getElementById('scalar-val');

        if (slider) {
            slider.addEventListener('input', (e) => {
                scalar = parseFloat(e.target.value);
                valDisplay.textContent = scalar.toFixed(1);
                canvas.draw();
            });
        }

        const baseDraw = canvas.draw.bind(canvas);

        canvas.draw = function () {
            // Draw grid & original vector (faint)
            baseDraw();

            // Calculate scaled vector
            const scaledV = new Vector2D(v.x * scalar, v.y * scalar);

            // Determine color based on scalar
            let color = '#ec4899'; // Pink/Purple default
            if (scalar < 0) color = '#ef4444'; // Red for negative

            const origin = this.toCanvas(0, 0);
            const end = this.toCanvas(scaledV.x, scaledV.y);

            this.drawArrow(origin, end, color, `${scalar}v`);
        };

        canvas.draw();
    })();

};
