/**
 * Chapter 7 Scripts: Quadratic Forms
 */
window.chapterScripts['ch7-quadratic'] = function () {
    window.logDebug && window.logDebug('Initializing Chapter 7');

    const canvas = new CanvasSpace('viz-quadratic', { interactive: true, scale: 50 });

    // Matrix Inputs: [a b; b d] (Symmetric)
    const inputs = {
        a: document.getElementById('q-a'),
        b: document.getElementById('q-b'),
        c: document.getElementById('q-c'), // mirror of b
        d: document.getElementById('q-d')
    };

    // Sync b and c
    inputs.b.addEventListener('input', () => { inputs.c.value = inputs.b.value; update(); });
    inputs.a.addEventListener('input', () => update());
    inputs.d.addEventListener('input', () => update());

    let m = { a: 2, b: 0, d: 1 };

    const update = () => {
        m.a = parseFloat(inputs.a.value) || 0;
        m.b = parseFloat(inputs.b.value) || 0;
        m.d = parseFloat(inputs.d.value) || 0;
        inputs.c.value = m.b;

        canvas.draw();
    };

    canvas.draw = function () {
        this.clear();
        this.drawGrid();
        const ctx = this.ctx;

        // Draw the contour x^T A x = 1 => ax^2 + 2bxy + dy^2 = 1
        // We can draw this by iterating angles, calculating radius r
        // let x = r cos theta, y = r sin theta
        // a(r cos)^2 + 2b(r cos)(r sin) + d(r sin)^2 = 1
        // r^2 (a cos^2 + 2b cos sin + d sin^2) = 1
        // r = 1 / sqrt( ... )

        ctx.beginPath();
        ctx.strokeStyle = '#4a9eff';
        ctx.lineWidth = 3;

        let started = false;

        for (let theta = 0; theta <= Math.PI * 2; theta += 0.05) {
            const cos = Math.cos(theta);
            const sin = Math.sin(theta);

            const denom = m.a * cos * cos + 2 * m.b * cos * sin + m.d * sin * sin;

            if (denom > 0) {
                // Real radius exists
                const r = 1 / Math.sqrt(denom);
                // Limit HUGE radius for rendering safety
                if (r > 20) {
                    // stroke current path and restart?
                    if (started) { ctx.stroke(); started = false; }
                    continue;
                }

                const pt = this.toCanvas(r * cos, r * sin);
                if (!started) {
                    ctx.beginPath();
                    ctx.moveTo(pt.x, pt.y);
                    started = true;
                } else {
                    ctx.lineTo(pt.x, pt.y);
                }
            } else {
                // No real solution in this direction (Hyperbola gaps)
                if (started) {
                    ctx.stroke();
                    started = false;
                }
            }
        }
        if (started) ctx.stroke();

        // Draw Eigenvectors (Axes of the ellipse)
        const matrix = new Matrix(2, 2, [[m.a, m.b], [m.b, m.d]]);
        const { eigenvalues, eigenvectors } = matrix.eigenvectors2x2();

        // eigenvectors is array of [x, y] arrays
        // eigenvalues is array of numbers

        // Draw axes
        eigenvectors.forEach((v, i) => {
            if (v) {
                // If eigenvalue is positive, this axis intersects the ellipse
                // Length of axis = 1/sqrt(lambda)
                const lambda = eigenvalues[i];
                let len = 3; // default drawing length
                if (lambda > 0) {
                    len = 1 / Math.sqrt(lambda);
                }

                const vec = new Vector2D(v[0], v[1]).scale(len);
                const end = this.toCanvas(vec);
                const start = this.toCanvas(0, 0);

                this.drawArrow(start, end, '#ef4444', `λ=${lambda.toFixed(2)}`);
            }
        });
    };

    update();
};
