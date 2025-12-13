/**
 * Chapter 10: Cross Products
 * Interactive 3D visualization of cross product with right-hand rule
 */
window.chapterScripts['e10-cross-product'] = function () {
    console.log('Initializing Chapter 10: Cross Product');

    (function initCrossProductDemo() {
        const canvas = document.getElementById('viz-cross-product');
        if (!canvas) return;

        const ctx = canvas.getContext('2d');
        const width = canvas.width = canvas.offsetWidth;
        const height = canvas.height = canvas.offsetHeight;
        const cx = width / 2;
        const cy = height / 2;
        const scale = 50;

        let angleX = 0.4;
        let angleY = 0.3;
        let isDragging = false;
        let lastX, lastY;

        // Vectors in 3D
        const v = { x: 2, y: 0, z: 0 };
        const w = { x: 0, y: 2, z: 0 };

        function crossProduct(a, b) {
            return {
                x: a.y * b.z - a.z * b.y,
                y: a.z * b.x - a.x * b.z,
                z: a.x * b.y - a.y * b.x
            };
        }

        function project(x, y, z) {
            const cosX = Math.cos(angleX), sinX = Math.sin(angleX);
            const cosY = Math.cos(angleY), sinY = Math.sin(angleY);

            const y1 = y * cosX - z * sinX;
            const z1 = y * sinX + z * cosX;
            const x2 = x * cosY + z1 * sinY;

            return {
                x: cx + x2 * scale,
                y: cy - y1 * scale
            };
        }

        function drawArrow3D(from, to, color, label) {
            const p1 = project(from.x, from.y, from.z);
            const p2 = project(to.x, to.y, to.z);

            ctx.beginPath();
            ctx.moveTo(p1.x, p1.y);
            ctx.lineTo(p2.x, p2.y);
            ctx.strokeStyle = color;
            ctx.lineWidth = 3;
            ctx.stroke();

            // Arrowhead
            const angle = Math.atan2(p2.y - p1.y, p2.x - p1.x);
            const headLen = 10;
            ctx.beginPath();
            ctx.moveTo(p2.x, p2.y);
            ctx.lineTo(p2.x - headLen * Math.cos(angle - 0.4), p2.y - headLen * Math.sin(angle - 0.4));
            ctx.lineTo(p2.x - headLen * Math.cos(angle + 0.4), p2.y - headLen * Math.sin(angle + 0.4));
            ctx.closePath();
            ctx.fillStyle = color;
            ctx.fill();

            // Label
            ctx.fillStyle = color;
            ctx.font = 'bold 14px sans-serif';
            ctx.fillText(label, p2.x + 5, p2.y - 5);
        }

        function draw() {
            ctx.clearRect(0, 0, width, height);

            // Axes
            const origin = { x: 0, y: 0, z: 0 };
            drawArrow3D(origin, { x: 3, y: 0, z: 0 }, 'rgba(150,150,150,0.5)', 'X');
            drawArrow3D(origin, { x: 0, y: 3, z: 0 }, 'rgba(150,150,150,0.5)', 'Y');
            drawArrow3D(origin, { x: 0, y: 0, z: 3 }, 'rgba(150,150,150,0.5)', 'Z');

            // Vectors
            drawArrow3D(origin, v, '#4a9eff', 'v');
            drawArrow3D(origin, w, '#f97316', 'w');

            // Cross product
            const cross = crossProduct(v, w);
            drawArrow3D(origin, cross, '#10b981', 'v × w');

            // Parallelogram
            const p0 = project(0, 0, 0);
            const pv = project(v.x, v.y, v.z);
            const pw = project(w.x, w.y, w.z);
            const pvw = project(v.x + w.x, v.y + w.y, v.z + w.z);

            ctx.beginPath();
            ctx.moveTo(p0.x, p0.y);
            ctx.lineTo(pv.x, pv.y);
            ctx.lineTo(pvw.x, pvw.y);
            ctx.lineTo(pw.x, pw.y);
            ctx.closePath();
            ctx.fillStyle = 'rgba(236, 72, 153, 0.2)';
            ctx.fill();
            ctx.strokeStyle = 'rgba(236, 72, 153, 0.5)';
            ctx.lineWidth = 1;
            ctx.stroke();

            // Info
            const mag = Math.sqrt(cross.x * cross.x + cross.y * cross.y + cross.z * cross.z);
            ctx.fillStyle = '#888';
            ctx.font = '14px sans-serif';
            ctx.fillText(`|v × w| = ${mag.toFixed(2)} (面积)`, 20, 30);
            ctx.fillText('拖拽旋转视角', 20, height - 20);
        }

        canvas.addEventListener('mousedown', (e) => {
            isDragging = true;
            lastX = e.clientX;
            lastY = e.clientY;
        });

        canvas.addEventListener('mousemove', (e) => {
            if (!isDragging) return;
            const dx = e.clientX - lastX;
            const dy = e.clientY - lastY;
            angleY += dx * 0.01;
            angleX += dy * 0.01;
            lastX = e.clientX;
            lastY = e.clientY;
            draw();
        });

        canvas.addEventListener('mouseup', () => isDragging = false);
        canvas.addEventListener('mouseleave', () => isDragging = false);

        draw();
    })();
};
