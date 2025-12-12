/**
 * Chapter 4 Scripts: Determinants
 */
window.chapterScripts['ch4-determinants'] = function () {
    window.logDebug && window.logDebug('Initializing Chapter 4');

    const canvas = new CanvasSpace('viz-determinant', { interactive: true, scale: 50 });

    // Matrix state
    const matInput = {
        a: document.getElementById('det-m00'),
        b: document.getElementById('det-m01'),
        c: document.getElementById('det-m10'),
        d: document.getElementById('det-m11')
    };

    // Current matrix
    let m = { a: 1, b: 0, c: 0, d: 1 };

    const updateInputs = () => {
        m.a = parseFloat(matInput.a.value) || 0;
        m.b = parseFloat(matInput.b.value) || 0;
        m.c = parseFloat(matInput.c.value) || 0;
        m.d = parseFloat(matInput.d.value) || 0;

        // Update math display
        const ad = m.a * m.d;
        const bc = m.b * m.c;
        const det = ad - bc;

        document.getElementById('calc-ad').textContent = (m.a * m.d).toFixed(2);
        document.getElementById('calc-bc').textContent = (m.b * m.c).toFixed(2);

        const detEl = document.getElementById('val-det');
        detEl.textContent = det.toFixed(2);

        if (det < 0) detEl.style.color = '#ef4444'; // Red for negative
        else detEl.style.color = '#10b981'; // Green for positive

        canvas.draw();
    };

    // Add event listeners
    Object.values(matInput).forEach(input => {
        if (input) input.addEventListener('input', updateInputs);
    });

    // Override draw
    canvas.draw = function () {
        this.clear();
        this.drawGrid();

        const ctx = this.ctx;
        const origin = this.options.origin;

        // Vertices of unit square: (0,0), (1,0), (1,1), (0,1)
        // Apply transform
        const transform = (x, y) => {
            return {
                x: m.a * x + m.b * y,
                y: m.c * x + m.d * y
            };
        };

        const p0 = transform(0, 0);
        const p1 = transform(1, 0);
        const p2 = transform(1, 1);
        const p3 = transform(0, 1);

        // Convert to canvas coords
        const cp0 = this.toCanvas(p0);
        const cp1 = this.toCanvas(p1);
        const cp2 = this.toCanvas(p2);
        const cp3 = this.toCanvas(p3);

        // Draw Fill
        ctx.beginPath();
        ctx.moveTo(cp0.x, cp0.y);
        ctx.lineTo(cp1.x, cp1.y);
        ctx.lineTo(cp2.x, cp2.y);
        ctx.lineTo(cp3.x, cp3.y);
        ctx.closePath();

        const det = m.a * m.d - m.b * m.c;

        if (det < 0) {
            ctx.fillStyle = 'rgba(239, 68, 68, 0.3)'; // Redish for negative (flipped)
        } else {
            ctx.fillStyle = 'rgba(16, 185, 129, 0.3)'; // Greenish for positive
        }
        ctx.fill();
        ctx.strokeStyle = '#fff';
        ctx.lineWidth = 1;
        ctx.stroke();

        // Draw Basis Vectors
        this.drawArrow(this.toCanvas(0, 0), cp1, '#f97316', det === 0 ? '' : 'i\'');
        this.drawArrow(this.toCanvas(0, 0), cp3, '#10b981', det === 0 ? '' : 'j\'');

        // Show unit square outline (ghost)
        const u1 = this.toCanvas(1, 0);
        const u2 = this.toCanvas(1, 1);
        const u3 = this.toCanvas(0, 1);

        ctx.save();
        ctx.setLineDash([5, 5]);
        ctx.strokeStyle = 'rgba(255, 255, 255, 0.2)';
        ctx.beginPath();
        ctx.moveTo(origin.x, origin.y);
        ctx.lineTo(u1.x, u1.y);
        ctx.lineTo(u2.x, u2.y);
        ctx.lineTo(u3.x, u3.y);
        ctx.closePath();
        ctx.stroke();
        ctx.restore();
    };

    // Initial draw
    updateInputs();
};
