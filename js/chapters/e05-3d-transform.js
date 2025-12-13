/**
 * Chapter 5: 3D Linear Transformations
 * Interactive 3D visualization using 2D projection
 */
window.chapterScripts['e05-3d-transform'] = function () {
    console.log('Initializing Chapter 5: 3D Transformations');

    // Simple 3D cube projected to 2D
    (function init3DDemo() {
        const canvas = document.getElementById('viz-3d-transform');
        if (!canvas) return;

        const ctx = canvas.getContext('2d');
        const width = canvas.width = canvas.offsetWidth;
        const height = canvas.height = canvas.offsetHeight;
        const cx = width / 2;
        const cy = height / 2;
        const scale = 60;

        // Cube vertices
        const cubeVertices = [
            [-1, -1, -1], [1, -1, -1], [1, 1, -1], [-1, 1, -1],
            [-1, -1, 1], [1, -1, 1], [1, 1, 1], [-1, 1, 1]
        ];

        const edges = [
            [0, 1], [1, 2], [2, 3], [3, 0],
            [4, 5], [5, 6], [6, 7], [7, 4],
            [0, 4], [1, 5], [2, 6], [3, 7]
        ];

        let angleX = 0.5;
        let angleY = 0.5;
        let isDragging = false;
        let lastX, lastY;

        function project(x, y, z) {
            // Simple isometric projection
            const cosX = Math.cos(angleX), sinX = Math.sin(angleX);
            const cosY = Math.cos(angleY), sinY = Math.sin(angleY);

            // Rotate around X axis
            const y1 = y * cosX - z * sinX;
            const z1 = y * sinX + z * cosX;

            // Rotate around Y axis
            const x2 = x * cosY + z1 * sinY;
            const z2 = -x * sinY + z1 * cosY;

            return {
                x: cx + x2 * scale,
                y: cy - y1 * scale
            };
        }

        function draw() {
            ctx.clearRect(0, 0, width, height);

            // Draw axes
            const origin = project(0, 0, 0);
            const xAxis = project(2, 0, 0);
            const yAxis = project(0, 2, 0);
            const zAxis = project(0, 0, 2);

            // X axis (red)
            ctx.beginPath();
            ctx.moveTo(origin.x, origin.y);
            ctx.lineTo(xAxis.x, xAxis.y);
            ctx.strokeStyle = '#f97316';
            ctx.lineWidth = 3;
            ctx.stroke();

            // Y axis (green)
            ctx.beginPath();
            ctx.moveTo(origin.x, origin.y);
            ctx.lineTo(yAxis.x, yAxis.y);
            ctx.strokeStyle = '#10b981';
            ctx.lineWidth = 3;
            ctx.stroke();

            // Z axis (blue)
            ctx.beginPath();
            ctx.moveTo(origin.x, origin.y);
            ctx.lineTo(zAxis.x, zAxis.y);
            ctx.strokeStyle = '#4a9eff';
            ctx.lineWidth = 3;
            ctx.stroke();

            // Draw cube
            ctx.strokeStyle = 'rgba(139, 92, 246, 0.8)';
            ctx.lineWidth = 2;
            edges.forEach(([i, j]) => {
                const p1 = project(...cubeVertices[i]);
                const p2 = project(...cubeVertices[j]);
                ctx.beginPath();
                ctx.moveTo(p1.x, p1.y);
                ctx.lineTo(p2.x, p2.y);
                ctx.stroke();
            });

            // Labels
            ctx.fillStyle = '#f97316';
            ctx.font = 'bold 14px sans-serif';
            ctx.fillText('î (X)', xAxis.x + 5, xAxis.y);
            ctx.fillStyle = '#10b981';
            ctx.fillText('ĵ (Y)', yAxis.x + 5, yAxis.y);
            ctx.fillStyle = '#4a9eff';
            ctx.fillText('k̂ (Z)', zAxis.x + 5, zAxis.y);
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
