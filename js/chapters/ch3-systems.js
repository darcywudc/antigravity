/**
 * Chapter 3 Scripts: Linear Systems
 */
window.chapterScripts['ch3-systems'] = function () {
    window.logDebug && window.logDebug('Initializing Chapter 3 Logic');

    // Check canvas
    const canvasEl = document.getElementById('viz-systems');
    if (!canvasEl) {
        window.logDebug && window.logDebug('Error: viz-systems canvas not found!', 'error');
        return;
    }

    // Check dimensions
    if (canvasEl.offsetWidth === 0) {
        window.logDebug && window.logDebug('Warning: Canvas width is 0!', 'error');
    }

    // Initialize Canvas
    // Use scale 30, and origin at (300, 400)
    // IMPORTANT: Custom draw function is needed for immediate mode drawing
    const canvas = new CanvasSpace('viz-systems', { interactive: true, scale: 30, origin: { x: 300, y: 400 }, autoCenter: false });

    // 方程组参数:
    // x - 2y = 1
    // 3x + 2y = 11
    // 解: x = 3, y = 1

    let mode = 'row'; // 'row' or 'col'

    const drawRowPicture = () => {
        // We do NOT call canvas.clear() here because canvas.draw() already calls it
        // But if we call this directly, we might need to clear.
        // Better pattern: Let canvas.draw handle clearing.

        canvas.drawGrid();

        const ctx = canvas.ctx;

        // 绘制第一条线: x - 2y = 1 => y = 0.5x - 0.5
        const p1_start = canvas.toCanvas(-10, 0.5 * -10 - 0.5);
        const p1_end = canvas.toCanvas(10, 0.5 * 10 - 0.5);

        ctx.beginPath();
        ctx.strokeStyle = '#ef4444'; // Red
        ctx.lineWidth = 2;
        ctx.moveTo(p1_start.x, p1_start.y);
        ctx.lineTo(p1_end.x, p1_end.y);
        ctx.stroke();

        // 绘制第二条线: 3x + 2y = 11 => y = -1.5x + 5.5
        const p2_start = canvas.toCanvas(-2, -1.5 * -2 + 5.5);
        const p2_end = canvas.toCanvas(8, -1.5 * 8 + 5.5);

        ctx.beginPath();
        ctx.strokeStyle = '#3b82f6'; // Blue
        ctx.lineWidth = 2;
        ctx.moveTo(p2_start.x, p2_start.y);
        ctx.lineTo(p2_end.x, p2_end.y);
        ctx.stroke();

        // 绘制交点 (3, 1)
        const intersection = canvas.toCanvas(3, 1);
        ctx.beginPath();
        ctx.fillStyle = '#ffffff';
        ctx.arc(intersection.x, intersection.y, 5, 0, Math.PI * 2);
        ctx.fill();

        // 标注
        ctx.fillStyle = '#ef4444';
        ctx.fillText('x - 2y = 1', p1_end.x - 20, p1_end.y - 10);
        ctx.fillStyle = '#3b82f6';
        ctx.fillText('3x + 2y = 11', p2_end.x - 20, p2_end.y - 10);
        ctx.fillStyle = '#fff';
        ctx.fillText('(3, 1)', intersection.x + 10, intersection.y - 10);
    };

    const drawColPicture = () => {
        // canvas.clear() handled by canvas.draw() wrapper
        canvas.drawGrid();

        const c1 = new Vector2D(1, 3);
        const c2 = new Vector2D(-2, 2);
        // const b = new Vector2D(1, 11);
        const x = 3;
        const y = 1;

        const scaledC1 = c1.scale(x);
        const final = scaledC1.add(c2.scale(y));

        const start = canvas.toCanvas(0, 0);

        const endC1 = canvas.toCanvas(scaledC1.x, scaledC1.y);
        canvas.drawArrow(start, endC1, '#ef4444', '3 * col1');

        const endC2 = canvas.toCanvas(final.x, final.y);
        canvas.drawArrow(endC1, endC2, '#3b82f6', '1 * col2');

        canvas.drawArrow(start, endC2, '#fbbf24', 'b');
    };

    // Override draw method to persist graphics on resize/update
    canvas.draw = function () {
        this.clear();
        if (mode === 'row') {
            drawRowPicture();
        } else {
            drawColPicture();
        }
    };

    const updateUI = () => {
        if (mode === 'row') {
            const btnRow = document.getElementById('btn-mode-row');
            const btnCol = document.getElementById('btn-mode-col');
            if (btnRow) btnRow.className = 'btn btn-primary';
            if (btnCol) btnCol.className = 'btn btn-secondary';
            const instr = document.getElementById('viz-instruction');
            if (instr) instr.innerText = '行图像：每一行代表一条直线，解是它们的交点。';
        } else {
            const btnRow = document.getElementById('btn-mode-row');
            const btnCol = document.getElementById('btn-mode-col');
            if (btnRow) btnRow.className = 'btn btn-secondary';
            if (btnCol) btnCol.className = 'btn btn-primary';
            const instr = document.getElementById('viz-instruction');
            if (instr) instr.innerText = '列图像：解是列向量的线性组合系数 (x=3, y=1)，使得它们首尾相接到达 b。';
        }
        // Trigger redraw
        canvas.draw();
    };

    // Bind buttons
    const btnRow = document.getElementById('btn-mode-row');
    const btnCol = document.getElementById('btn-mode-col');

    if (btnRow && btnCol) {
        btnRow.addEventListener('click', () => { mode = 'row'; updateUI(); });
        btnCol.addEventListener('click', () => { mode = 'col'; updateUI(); });

        // Initial Draw
        updateUI();
        window.logDebug('Ch3 initialized and drawn');
    } else {
        window.logDebug('Error: Ch3 Buttons not found in DOM', 'error');
    }
};
