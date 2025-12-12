/**
 * Chapter 6 Scripts: Orthogonality
 */
window.chapterScripts['ch6-orthogonality'] = function () {
    window.logDebug && window.logDebug('Initializing Chapter 6');

    const canvas = new CanvasSpace('viz-ortho', { interactive: true, scale: 60 });

    // Base vector a (fixed for simplicity, or draggable)
    // Let's make 'a' fixed on x-axis to start, but dragging 'a' is cooler.
    const a = new Vector2D(4, 1);
    const aObj = canvas.addVector(a, '#f97316', 'a');
    canvas.makeDraggable(aObj, () => updateProj());

    // Input vector b
    const b = new Vector2D(2, 3);
    const bObj = canvas.addVector(b, '#4a9eff', 'b');
    canvas.makeDraggable(bObj, () => updateProj());

    // Projection p (calculated)
    // We won't add it as a normal vector in `canvas.vectors` because we want to draw the dashed error line too.
    // We'll calculate it in draw override or update loop.

    const updateProj = () => {
        canvas.draw();
    };

    canvas.draw = function () {
        this.clear();
        this.drawGrid();

        const origin = this.options.origin;

        // Draw a and b standard
        const start = this.toCanvas(0, 0);
        const endA = this.toCanvas(a);
        const endB = this.toCanvas(b);

        this.drawArrow(start, endA, '#f97316', 'a');
        this.drawArrow(start, endB, '#4a9eff', 'b');

        // Calculate Projection
        // p = (b.a / a.a) * a
        const adota = a.dot(a);
        if (adota < 0.0001) return; // Avoid divide by zero

        const factor = b.dot(a) / adota;
        const p = a.scale(factor);

        const endP = this.toCanvas(p);

        // Draw P (Green)
        this.drawArrow(start, endP, '#10b981', 'p');

        // Draw Error e = b - p (Dashed line from P to B)
        const ctx = this.ctx;
        ctx.save();
        ctx.setLineDash([5, 5]);
        ctx.strokeStyle = '#fca5a5';
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(endP.x, endP.y);
        ctx.lineTo(endB.x, endB.y);
        ctx.stroke();
        ctx.restore();

        // Label e
        ctx.fillStyle = '#fca5a5';
        ctx.font = 'bold 14px sans-serif';
        ctx.fillText('e', (endP.x + endB.x) / 2 + 10, (endP.y + endB.y) / 2);

        // Verify orthogonality: a dot e should be 0
        const eVec = b.subtract(p);
        const dot = a.dot(eVec);

        document.getElementById('ortho-calc').innerHTML =
            `$$ \\mathbf{a} \\cdot \\mathbf{e} = ${dot.toFixed(5)} \\approx 0 $$`;

        // Render math if needed? 
    };

    updateProj();
};
