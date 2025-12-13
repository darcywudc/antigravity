/**
 * Chapter 8: Nonsquare Matrices
 * Visualization of dimension changes (2D to 3D and vice versa)
 */
window.chapterScripts['e08-nonsquare'] = function () {
    console.log('Initializing Chapter 8: Nonsquare Matrices');

    (function initDimensionDemo() {
        const canvas = document.getElementById('viz-nonsquare');
        if (!canvas) return;

        const ctx = canvas.getContext('2d');
        const width = canvas.width = canvas.offsetWidth;
        const height = canvas.height = canvas.offsetHeight;

        let mode = '2to3'; // or '3to2'
        let animProgress = 0;
        let animating = false;

        function draw2Dto3D() {
            ctx.clearRect(0, 0, width, height);

            const leftCx = width * 0.25;
            const rightCx = width * 0.75;
            const cy = height / 2;
            const scale = 40;

            // Left: 2D plane
            ctx.fillStyle = '#4a9eff';
            ctx.font = 'bold 14px sans-serif';
            ctx.fillText('2D 平面', leftCx - 30, 30);

            // Draw 2D point
            const p2d = { x: 1.5, y: 1 };
            const p2dCanvas = { x: leftCx + p2d.x * scale, y: cy - p2d.y * scale };

            // 2D axes
            ctx.strokeStyle = 'rgba(100,100,100,0.5)';
            ctx.beginPath();
            ctx.moveTo(leftCx - 80, cy);
            ctx.lineTo(leftCx + 80, cy);
            ctx.moveTo(leftCx, cy + 80);
            ctx.lineTo(leftCx, cy - 80);
            ctx.stroke();

            ctx.beginPath();
            ctx.arc(p2dCanvas.x, p2dCanvas.y, 8, 0, Math.PI * 2);
            ctx.fillStyle = '#4a9eff';
            ctx.fill();

            // Arrow showing transformation
            const arrowStart = { x: leftCx + 100, y: cy };
            const arrowEnd = { x: rightCx - 100, y: cy };
            ctx.beginPath();
            ctx.moveTo(arrowStart.x, arrowStart.y);
            ctx.lineTo(arrowEnd.x, arrowEnd.y);
            ctx.strokeStyle = '#10b981';
            ctx.lineWidth = 3;
            ctx.stroke();
            // Arrowhead
            ctx.beginPath();
            ctx.moveTo(arrowEnd.x, arrowEnd.y);
            ctx.lineTo(arrowEnd.x - 10, arrowEnd.y - 5);
            ctx.lineTo(arrowEnd.x - 10, arrowEnd.y + 5);
            ctx.closePath();
            ctx.fillStyle = '#10b981';
            ctx.fill();

            // Label
            ctx.fillStyle = '#10b981';
            ctx.fillText('3×2 矩阵', (leftCx + rightCx) / 2 - 30, cy - 20);

            // Right: 3D view (simple projection)
            ctx.fillStyle = '#f97316';
            ctx.fillText('3D 空间', rightCx - 30, 30);

            // 3D axes (simple)
            const angle = 0.5;
            ctx.strokeStyle = 'rgba(100,100,100,0.5)';
            ctx.beginPath();
            // X
            ctx.moveTo(rightCx, cy);
            ctx.lineTo(rightCx + 60, cy);
            // Y
            ctx.moveTo(rightCx, cy);
            ctx.lineTo(rightCx, cy - 60);
            // Z (isometric)
            ctx.moveTo(rightCx, cy);
            ctx.lineTo(rightCx - 40 * Math.cos(angle), cy + 40 * Math.sin(angle));
            ctx.stroke();

            // Embedded point in 3D
            const t = animProgress;
            const p3d = {
                x: p2d.x,
                y: p2d.y,
                z: (p2d.x + p2d.y) * t // z = x + y in the embedding
            };
            const p3dCanvas = {
                x: rightCx + p3d.x * scale * 0.6 - p3d.z * 15 * Math.cos(angle),
                y: cy - p3d.y * scale * 0.6 + p3d.z * 15 * Math.sin(angle)
            };

            ctx.beginPath();
            ctx.arc(p3dCanvas.x, p3dCanvas.y, 8, 0, Math.PI * 2);
            ctx.fillStyle = '#f97316';
            ctx.fill();

            ctx.fillStyle = '#888';
            ctx.font = '12px sans-serif';
            ctx.fillText(`(${p2d.x}, ${p2d.y}) → (${p2d.x}, ${p2d.y}, ${(p2d.x + p2d.y).toFixed(1)})`, rightCx - 60, height - 30);
        }

        function animate() {
            if (!animating) return;
            animProgress += 0.03;
            if (animProgress >= 1) {
                animProgress = 1;
                animating = false;
            }
            draw2Dto3D();
            if (animating) requestAnimationFrame(animate);
        }

        const embedBtn = document.getElementById('btn-embed');
        const projectBtn = document.getElementById('btn-project');

        if (embedBtn) {
            embedBtn.addEventListener('click', () => {
                mode = '2to3';
                animProgress = 0;
                animating = true;
                animate();
            });
        }

        draw2Dto3D();
    })();
};
