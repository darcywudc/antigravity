/**
 * Chapter 6: The Determinant
 * Interactive visualization showing area scaling
 */
window.chapterScripts['e06-determinant'] = function () {
    console.log('Initializing Chapter 6: Determinant');

    (function initDetDemo() {
        const canvas = document.getElementById('viz-determinant');
        if (!canvas) return;

        const ctx = canvas.getContext('2d');
        const width = canvas.width = canvas.offsetWidth;
        const height = canvas.height = canvas.offsetHeight;
        const cx = width / 2;
        const cy = height / 2;
        const scale = 40;

        // Matrix values (editable)
        let a = 2, b = 1, c = 0.5, d = 2;

        function toCanvas(x, y) {
            return { x: cx + x * scale, y: cy - y * scale };
        }

        function drawGrid() {
            ctx.strokeStyle = 'rgba(100, 100, 100, 0.3)';
            ctx.lineWidth = 1;
            for (let i = -10; i <= 10; i++) {
                const p1 = toCanvas(i, -10);
                const p2 = toCanvas(i, 10);
                ctx.beginPath();
                ctx.moveTo(p1.x, p1.y);
                ctx.lineTo(p2.x, p2.y);
                ctx.stroke();

                const p3 = toCanvas(-10, i);
                const p4 = toCanvas(10, i);
                ctx.beginPath();
                ctx.moveTo(p3.x, p3.y);
                ctx.lineTo(p4.x, p4.y);
                ctx.stroke();
            }
        }

        function draw() {
            ctx.clearRect(0, 0, width, height);
            drawGrid();

            // Original unit square (faded)
            const o = toCanvas(0, 0);
            const u1 = toCanvas(1, 0);
            const u2 = toCanvas(1, 1);
            const u3 = toCanvas(0, 1);

            ctx.fillStyle = 'rgba(100, 100, 100, 0.2)';
            ctx.beginPath();
            ctx.moveTo(o.x, o.y);
            ctx.lineTo(u1.x, u1.y);
            ctx.lineTo(u2.x, u2.y);
            ctx.lineTo(u3.x, u3.y);
            ctx.closePath();
            ctx.fill();

            // Transformed parallelogram
            const t0 = toCanvas(0, 0);
            const t1 = toCanvas(a, c);
            const t2 = toCanvas(a + b, c + d);
            const t3 = toCanvas(b, d);

            const det = a * d - b * c;
            const color = det > 0 ? 'rgba(16, 185, 129, 0.4)' :
                det < 0 ? 'rgba(239, 68, 68, 0.4)' : 'rgba(100, 100, 100, 0.4)';

            ctx.fillStyle = color;
            ctx.beginPath();
            ctx.moveTo(t0.x, t0.y);
            ctx.lineTo(t1.x, t1.y);
            ctx.lineTo(t2.x, t2.y);
            ctx.lineTo(t3.x, t3.y);
            ctx.closePath();
            ctx.fill();
            ctx.strokeStyle = det > 0 ? '#10b981' : det < 0 ? '#ef4444' : '#666';
            ctx.lineWidth = 2;
            ctx.stroke();

            // Basis vectors
            ctx.lineWidth = 3;
            // i-hat transformed
            ctx.beginPath();
            ctx.moveTo(t0.x, t0.y);
            ctx.lineTo(t1.x, t1.y);
            ctx.strokeStyle = '#f97316';
            ctx.stroke();
            // j-hat transformed
            ctx.beginPath();
            ctx.moveTo(t0.x, t0.y);
            ctx.lineTo(t3.x, t3.y);
            ctx.strokeStyle = '#4a9eff';
            ctx.stroke();

            // Update display
            const detDisplay = document.getElementById('det-value');
            if (detDisplay) {
                detDisplay.textContent = det.toFixed(2);
                detDisplay.style.color = det > 0 ? '#10b981' : det < 0 ? '#ef4444' : '#888';
            }

            const areaDisplay = document.getElementById('area-display');
            if (areaDisplay) {
                areaDisplay.textContent = Math.abs(det).toFixed(2);
            }
        }

        // Slider controls
        ['det-a', 'det-b', 'det-c', 'det-d'].forEach((id, i) => {
            const slider = document.getElementById(id);
            if (slider) {
                slider.addEventListener('input', (e) => {
                    const val = parseFloat(e.target.value);
                    if (i === 0) a = val;
                    else if (i === 1) b = val;
                    else if (i === 2) c = val;
                    else d = val;
                    draw();
                });
            }
        });

        draw();
    })();
};
