/**
 * Chapter 3: Linear Transformations
 */
window.chapterScripts['e03-transformations'] = function () {
    console.log('Initializing Chapter 3 (Linear Transformations)');

    const canvasEl = document.getElementById('viz-matrix-transform');
    if (!canvasEl) {
        console.warn('Canvas viz-matrix-transform not found');
        return;
    }

    const canvas = new CanvasSpace('viz-matrix-transform', { interactive: true });

    // Current transformation matrix
    let matrix = { a: 1, b: 0, c: 0, d: 1 };

    // Input elements
    const valA = document.getElementById('val-a');
    const valB = document.getElementById('val-b');
    const valC = document.getElementById('val-c');
    const valD = document.getElementById('val-d');

    // Update matrix from inputs
    function readMatrix() {
        matrix.a = parseFloat(valA?.value) || 0;
        matrix.b = parseFloat(valB?.value) || 0;
        matrix.c = parseFloat(valC?.value) || 0;
        matrix.d = parseFloat(valD?.value) || 0;
    }

    // Set matrix to inputs
    function writeMatrix() {
        if (valA) valA.value = matrix.a.toFixed(1);
        if (valB) valB.value = matrix.b.toFixed(1);
        if (valC) valC.value = matrix.c.toFixed(1);
        if (valD) valD.value = matrix.d.toFixed(1);
    }

    // Draw transformed grid
    canvas.draw = function () {
        this.clear();
        const ctx = this.ctx;
        const origin = this.toCanvas(0, 0);

        // Apply transformation to basis vectors
        const i_hat = { x: matrix.a, y: matrix.c };
        const j_hat = { x: matrix.b, y: matrix.d };

        // Draw transformed grid
        ctx.save();
        ctx.strokeStyle = 'rgba(74, 158, 255, 0.15)';
        ctx.lineWidth = 1;

        const range = 10;
        for (let k = -range; k <= range; k++) {
            // Lines parallel to j
            const s1 = this.toCanvas(k * i_hat.x - range * j_hat.x, k * i_hat.y - range * j_hat.y);
            const e1 = this.toCanvas(k * i_hat.x + range * j_hat.x, k * i_hat.y + range * j_hat.y);
            ctx.beginPath(); ctx.moveTo(s1.x, s1.y); ctx.lineTo(e1.x, e1.y); ctx.stroke();

            // Lines parallel to i
            const s2 = this.toCanvas(-range * i_hat.x + k * j_hat.x, -range * i_hat.y + k * j_hat.y);
            const e2 = this.toCanvas(range * i_hat.x + k * j_hat.x, range * i_hat.y + k * j_hat.y);
            ctx.beginPath(); ctx.moveTo(s2.x, s2.y); ctx.lineTo(e2.x, e2.y); ctx.stroke();
        }
        ctx.restore();

        // Draw transformed basis vectors
        const iEnd = this.toCanvas(i_hat.x, i_hat.y);
        const jEnd = this.toCanvas(j_hat.x, j_hat.y);

        this.drawArrow(origin, iEnd, '#f97316', 'î\'');
        this.drawArrow(origin, jEnd, '#10b981', 'ĵ\'');
    };

    // Input listeners
    [valA, valB, valC, valD].forEach(el => {
        if (el) {
            el.addEventListener('input', () => {
                readMatrix();
                canvas.draw();
            });
        }
    });

    // Preset buttons
    document.getElementById('btn-shear')?.addEventListener('click', () => {
        matrix = { a: 1, b: 1, c: 0, d: 1 };
        writeMatrix();
        canvas.draw();
    });

    document.getElementById('btn-rotate')?.addEventListener('click', () => {
        const angle = Math.PI / 4;
        matrix = {
            a: Math.cos(angle),
            b: -Math.sin(angle),
            c: Math.sin(angle),
            d: Math.cos(angle)
        };
        writeMatrix();
        canvas.draw();
    });

    document.getElementById('btn-scale')?.addEventListener('click', () => {
        matrix = { a: 2, b: 0, c: 0, d: 2 };
        writeMatrix();
        canvas.draw();
    });

    // Initial draw
    writeMatrix();
    canvas.draw();
};
