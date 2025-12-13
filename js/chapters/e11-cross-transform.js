/**
 * Chapter 11: Cross Products and Transformations
 * Interactive volume (triple product) visualization
 */
window.chapterScripts['e11-cross-transform'] = function () {
    console.log('Initializing Chapter 11: Cross Product Transformations');

    (function initTripleProductDemo() {
        const canvas = document.getElementById('viz-triple-product');
        if (!canvas) return;

        const ctx = canvas.getContext('2d');
        const width = canvas.width = canvas.offsetWidth;
        const height = canvas.height = canvas.offsetHeight;
        const cx = width / 2;
        const cy = height / 2;
        const scale = 35;

        let angleX = 0.5;
        let angleY = 0.4;
        let isDragging = false;
        let lastX, lastY;

        // Three vectors forming a parallelepiped
        const v = { x: 2, y: 0, z: 0 };
        const w = { x: 0.5, y: 2, z: 0 };
        const u = { x: 0.3, y: 0.5, z: 1.5 };

        function crossProduct(a, b) {
            return {
                x: a.y * b.z - a.z * b.y,
                y: a.z * b.x - a.x * b.z,
                z: a.x * b.y - a.y * b.x
            };
        }

        function dotProduct(a, b) {
            return a.x * b.x + a.y * b.y + a.z * b.z;
        }

        function project(x, y, z) {
            const cosX = Math.cos(angleX), sinX = Math.sin(angleX);
            const cosY = Math.cos(angleY), sinY = Math.sin(angleY);

            const y1 = y * cosX - z * sinX;
            const z1 = y * sinX + z * cosX;
            const x2 = x * cosY + z1 * sinY;

            return { x: cx + x2 * scale, y: cy - y1 * scale };
        }

        function drawLine3D(from, to, color, width = 2) {
            const p1 = project(from.x, from.y, from.z);
            const p2 = project(to.x, to.y, to.z);
            ctx.beginPath();
            ctx.moveTo(p1.x, p1.y);
            ctx.lineTo(p2.x, p2.y);
            ctx.strokeStyle = color;
            ctx.lineWidth = width;
            ctx.stroke();
        }

        function draw() {
            ctx.clearRect(0, 0, width, height);

            const origin = { x: 0, y: 0, z: 0 };

            // Draw parallelepiped edges
            const points = [
                origin,
                v,
                { x: v.x + w.x, y: v.y + w.y, z: v.z + w.z },
                w,
                u,
                { x: u.x + v.x, y: u.y + v.y, z: u.z + v.z },
                { x: u.x + v.x + w.x, y: u.y + v.y + w.y, z: u.z + v.z + w.z },
                { x: u.x + w.x, y: u.y + w.y, z: u.z + w.z }
            ];

            // Bottom face
            ctx.fillStyle = 'rgba(236, 72, 153, 0.15)';
            ctx.beginPath();
            const p0 = project(points[0].x, points[0].y, points[0].z);
            const p1 = project(points[1].x, points[1].y, points[1].z);
            const p2 = project(points[2].x, points[2].y, points[2].z);
            const p3 = project(points[3].x, points[3].y, points[3].z);
            ctx.moveTo(p0.x, p0.y);
            ctx.lineTo(p1.x, p1.y);
            ctx.lineTo(p2.x, p2.y);
            ctx.lineTo(p3.x, p3.y);
            ctx.closePath();
            ctx.fill();

            // Edges
            const edgeColor = 'rgba(139, 92, 246, 0.7)';
            // Bottom
            drawLine3D(points[0], points[1], edgeColor);
            drawLine3D(points[1], points[2], edgeColor);
            drawLine3D(points[2], points[3], edgeColor);
            drawLine3D(points[3], points[0], edgeColor);
            // Top
            drawLine3D(points[4], points[5], edgeColor);
            drawLine3D(points[5], points[6], edgeColor);
            drawLine3D(points[6], points[7], edgeColor);
            drawLine3D(points[7], points[4], edgeColor);
            // Verticals
            drawLine3D(points[0], points[4], edgeColor);
            drawLine3D(points[1], points[5], edgeColor);
            drawLine3D(points[2], points[6], edgeColor);
            drawLine3D(points[3], points[7], edgeColor);

            // Main vectors
            drawLine3D(origin, v, '#4a9eff', 3);
            drawLine3D(origin, w, '#f97316', 3);
            drawLine3D(origin, u, '#10b981', 3);

            // Labels
            const pv = project(v.x, v.y, v.z);
            const pw = project(w.x, w.y, w.z);
            const pu = project(u.x, u.y, u.z);
            ctx.font = 'bold 14px sans-serif';
            ctx.fillStyle = '#4a9eff';
            ctx.fillText('v', pv.x + 5, pv.y);
            ctx.fillStyle = '#f97316';
            ctx.fillText('w', pw.x + 5, pw.y);
            ctx.fillStyle = '#10b981';
            ctx.fillText('u', pu.x + 5, pu.y);

            // Volume calculation
            const cross = crossProduct(v, w);
            const volume = Math.abs(dotProduct(u, cross));

            ctx.fillStyle = '#fff';
            ctx.font = 'bold 16px sans-serif';
            ctx.fillText(`体积 = |u · (v × w)| = ${volume.toFixed(2)}`, 20, 30);
            ctx.font = '12px sans-serif';
            ctx.fillStyle = '#888';
            ctx.fillText('拖拽旋转视角', 20, height - 20);
        }

        canvas.addEventListener('mousedown', (e) => {
            isDragging = true;
            lastX = e.clientX;
            lastY = e.clientY;
        });

        canvas.addEventListener('mousemove', (e) => {
            if (!isDragging) return;
            angleY += (e.clientX - lastX) * 0.01;
            angleX += (e.clientY - lastY) * 0.01;
            lastX = e.clientX;
            lastY = e.clientY;
            draw();
        });

        canvas.addEventListener('mouseup', () => isDragging = false);
        canvas.addEventListener('mouseleave', () => isDragging = false);

        draw();
    })();
};
